"""Read verified running F4 image and barometer/IMU snapshots through SWD.

Read-only HOTPLUG access; no reset, halt, programming or memory writes.
Foreground publication continues during each dump: fields are diagnostic
snapshots, not an atomic sample. Use serial rows for a coherent height trace
and long counter intervals for rates, not cross-field equality assertions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
CLI=Path(r'D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe')


def hex_bytes(path):
    data={}; upper=0
    for line in path.read_text().splitlines():
        record=bytes.fromhex(line[1:])
        if sum(record)&255: raise ValueError('Invalid HEX checksum')
        count,kind=record[0],record[3]
        if kind==4: upper=int.from_bytes(record[4:6],'big')<<16
        elif kind==0:
            start=upper+int.from_bytes(record[1:3],'big')
            for i,value in enumerate(record[4:4+count]): data[start+i]=value
        elif kind==1: break
    return data


def upload(address,length,path,serial):
    result=subprocess.run([str(CLI),'-c','port=SWD',f'sn={serial}','freq=1000',
        'mode=HOTPLUG','-u',hex(address),str(length),str(path)],capture_output=True,
        timeout=60,creationflags=subprocess.CREATE_NO_WINDOW)
    path.with_suffix('.read.log').write_bytes(result.stdout+result.stderr)
    if result.returncode or not path.exists() or path.stat().st_size!=length:
        raise RuntimeError('SWD read failed: '+str(path))
    return path.read_bytes()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target',default='F4_BARO_TEST')
    parser.add_argument('--tag',required=True)
    parser.add_argument('--samples',type=int,default=3)
    parser.add_argument('--interval',type=float,default=5)
    parser.add_argument('--serial',default='35FF72063448303417211657')
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_]+',args.target) or not re.fullmatch(r'[A-Za-z0-9_]+',args.tag):
        parser.error('target and tag must be simple names')
    if not 1<=args.samples<=100 or not 0<=args.interval<=30: parser.error('invalid sampling options')
    folder=ROOT/'MDK-ARM'/args.target
    image=hex_bytes(folder/(args.target+'.hex'))
    start,end=min(image),max(image)+1
    flash=upload(start,end-start,ROOT/'Logs'/f'{args.tag}_flash.bin',args.serial)
    if any(flash[address-start]!=value for address,value in image.items()):
        raise RuntimeError('Running image differs from selected map/HEX; refuse to decode SRAM')
    map_text=(folder/(args.target+'.map')).read_text(errors='replace')
    names=['g_baro_status','g_baro_sample','g_imu_status','g_mag_status','g_spi_bus_status',
           'g_attitude','g_baro_pressure_valid','g_baro_altitude_valid']
    symbols={}
    for name in names:
        match=re.search(r'^\s+'+name+r'\s+(0x[0-9A-Fa-f]+)\s+Data\s+(\d+)',map_text,re.M)
        if not match: raise RuntimeError('Missing symbol '+name)
        symbols[name]=(int(match[1],16),int(match[2]))
    base=min(a for a,n in symbols.values()); size=max(a+n for a,n in symbols.values())-base
    if not 0x20000000<=base<base+size<=0x20030000: raise RuntimeError('Invalid SRAM range')
    output={'target':args.target,'verified_flash_bytes':len(image),'flash_sha256':hashlib.sha256(flash).hexdigest(),
            'atomic_snapshot':False,
            'symbols':symbols,'snapshots':[]}
    for index in range(args.samples):
        if index: time.sleep(args.interval)
        raw=upload(base,size,ROOT/'Logs'/f'{args.tag}_{index}.bin',args.serial)
        def unpack(name,fmt):
            address,length=symbols[name]
            if struct.calcsize(fmt)>length: raise RuntimeError('Layout larger than symbol '+name)
            return struct.unpack_from(fmt,raw,address-base)
        def fields(name,fmt,labels): return dict(zip(labels,unpack(name,fmt)))
        snapshot={'host_monotonic':time.monotonic(),
            'baro':fields('g_baro_status','<13I8Hf',[
                'state','error','model','samples','bus_errors','busy_skips','last_sample_us',
                'max_interval_us','reference_samples','reference_resets','crc_expected','crc_actual','flags',
                *[f'prom{i}' for i in range(8)],'reference_pressure_pa']),
            'sample':fields('g_baro_sample','<5I4i2f',[
                'timestamp_us','sequence','flags','d1','d2','pressure_pa','temperature_centi_c',
                'pressure_ms5611_pa','pressure_ms5607_pa','relative_altitude_m','filtered_altitude_m']),
            'imu':fields('g_imu_status','<6B2x9I',[
                'state','error','who_am_i','bad_register','readback_expected','readback_actual',
                'drdy_count','samples','busy_drops','queue_drops','invalid_samples','spi_errors','timeouts',
                'last_sample_us','max_transfer_us']),
            'mag':fields('g_mag_status','<14I',[
                'state','error','who_am_i','bad_register','expected','actual','samples','not_ready','overruns',
                'bus_errors','invalid_samples','last_sample_us','max_interval_us','calibration_valid']),
            'spi':fields('g_spi_bus_status','<6I',[
                'transactions','busy_skips','errors','deferred_drdy','lost_drdy','max_transfer_us']),
            'attitude':fields('g_attitude','<5I',['timestamp_us','sequence','state','error','flags']),
            'pressure_valid':unpack('g_baro_pressure_valid','<I')[0],
            'altitude_valid':unpack('g_baro_altitude_valid','<I')[0]}
        output['snapshots'].append(snapshot)
        (ROOT/'Logs'/f'{args.tag}.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(snapshot),flush=True)


if __name__=='__main__': main()
