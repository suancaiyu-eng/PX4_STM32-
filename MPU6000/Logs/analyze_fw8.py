"""Read-only validation of FW8 UART, SWD, calibration and Flash evidence."""
from pathlib import Path
import hashlib
import json
import math
import statistics
import struct
from analyze_fw7 import snapshot as old_snapshot

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT/'Logs'


def capture(name, boot=False):
    text = (LOGS/name).read_text(encoding='utf-8-sig')
    if boot:
        point = text.rfind('#BOOT MPU6000 FW8 ')
        assert point >= 0 and 'FireWater18 AHRS9' in text[point:point+100]
        text = text[point:]
    lines = text.splitlines()
    rows, diagnostics, fragments = [], [], []
    for index, line in enumerate(lines):
        if not line:
            continue
        if line.startswith('#'):
            diagnostics.append(line)
            continue
        parts = line[8:].split(',') if line.startswith('mpu6000:') else []
        if len(parts) != 18:
            assert index in (0, len(lines)-1), (name, index, line)
            fragments.append(line)
            continue
        # A partial last field must not masquerade as a complete row.
        if any(parts[i] not in ('0', '1') for i in [10,14,15,16,17]):
            assert index == len(lines)-1, (name,index,line)
            fragments.append(line)
            continue
        values = [int(value) for value in parts[:7]] + [float(v) for v in parts[7:]]
        assert all(-32768 <= v <= 32767 for v in values[:7])
        assert all(math.isfinite(v) for v in values)
        assert all(abs(v) <= 180 for v in values[7:10])
        assert values[14] == 0 or all(values[i] == 1 for i in [10,15,16,17])
        rows.append(values)
    return rows, diagnostics, fragments


def snapshot(tag):
    result = old_snapshot(tag)
    metadata = json.loads((LOGS/(tag+'.json')).read_text(encoding='utf-8-sig'))
    data = (LOGS/(tag+'.bin')).read_bytes()
    def off(name):
        return metadata['symbols'][name]['address'] - metadata['base']
    pose = result['attitude']
    pose.update(zip(['heading_state','mag_samples','mag_corrections','mag_rejections',
                     'mag_stale_events','mag_timestamp','mag_sequence'],
                    struct.unpack_from('<7I',data,off('g_attitude')+120)))
    fields = struct.unpack_from('<8f',data,off('g_attitude')+148)
    pose.update(magnetic_body_uT=fields[:3], magnetic_norm_uT=fields[3],
                magnetic_innovation_rad=fields[4], magnetic_bias_rad_s=fields[5:])
    p = off('g_mag_status')
    mag = dict(zip(['state','error','id','bad_register','expected','actual','samples',
                    'not_ready','overruns','bus_errors','invalid_samples','timestamp',
                    'max_interval_us','calibration_valid'],struct.unpack_from('<14I',data,p)))
    mag['raw'] = struct.unpack_from('<3i',data,p+56)
    mag['field_uT'] = struct.unpack_from('<3f',data,p+68)
    mag['norm_uT'],mag['sample_valid'],mag['calibration_error'] = struct.unpack_from('<fII',data,p+80)
    result['mag'] = mag
    result['bus'] = dict(zip(['transactions','busy_skips','errors','deferred_drdy',
                              'lost_drdy','max_transfer_us'],
                             struct.unpack_from('<6I',data,off('g_spi_bus_status'))))
    return result


def healthy(state):
    imu, pose, mag, bus, uart = [state[k] for k in ['driver','attitude','mag','bus','uart']]
    assert (imu['state'],imu['error'],imu['id']) == (6,0,0x68)
    assert all(imu[k] == 0 for k in ['busy_drops','queue_drops','invalid_samples','spi_errors','timeouts'])
    assert (pose['state'],pose['error'],pose['heading_state']) == (2,0,2)
    assert pose['flags'] & 117 == 117 and not pose['flags'] & 2
    assert pose['timing_faults'] == pose['gap_faults'] == 0
    assert abs(sum(v*v for v in pose['q'])-1) < 1e-5
    assert (mag['state'],mag['error'],mag['id']) == (2,0,0x49)
    assert (mag['calibration_valid'],mag['calibration_error'],mag['sample_valid']) == (1,0,1)
    assert mag['bus_errors'] == mag['invalid_samples'] == 0
    assert bus['errors'] == bus['lost_drdy'] == 0
    assert bus['max_transfer_us'] < 250 and imu['max_dma_us'] < 500
    assert all(uart[k] == 0 for k in ['errors','timeouts','start_errors','aborts','abort_timeouts','faulted'])
    return state


def main():
    a,b = [healthy(snapshot('fw8_final_'+v)) for v in ['A','B']]
    elapsed = ((b['attitude']['timestamp']-a['attitude']['timestamp']) & 0xffffffff)*1e-6
    rates = {key: ((b['attitude'][key]-a['attitude'][key]) & 0xffffffff)/elapsed
             for key in ['predictions','corrections','publications','mag_samples','mag_corrections']}
    assert 995 < rates['predictions'] < 1005
    assert 199 < rates['publications'] < 201 and 199 < rates['corrections'] < 201
    assert 95 < rates['mag_samples'] < 105 and 95 < rates['mag_corrections'] < 105
    assert a['uart']['dropped'] == b['uart']['dropped']
    rows, messages, fragments = capture('fw8_final_60s_20261019.txt')
    assert len(rows)>5900 and not messages
    assert all(all(row[i] == 1 for i in [10,14,15,16,17]) for row in rows)
    angles = {}
    for index,name in enumerate(['roll','pitch','yaw'],7):
        values = [row[index] for row in rows]
        angles[name] = dict(min=min(values),max=max(values),mean=statistics.mean(values),
                            stdev=statistics.pstdev(values),first=values[0],last=values[-1],
                            end_change=math.remainder(values[-1]-values[0],360))
    warm=[]
    for i in [1,2]:
        values, diagnostics, _ = capture(f'fw8_warm_reset_0{i}.txt',boot=True)
        assert any(row[10] == 0 for row in values)
        valid = sum(all(row[j] == 1 for j in [10,14,15,16,17]) for row in values)
        assert valid>100
        assert any('#ATT state=1' in msg for msg in diagnostics)
        assert not any('#IMU state=7' in msg or '#ATT state=3' in msg for msg in diagnostics)
        warm.append(dict(round=i, valid_records=valid))
    fault = snapshot('fw8_mag_fault_state')
    assert (fault['mag']['state'],fault['driver']['state'],fault['attitude']['state']) == (3,6,2)
    assert fault['attitude']['flags'] & 1 and not fault['attitude']['flags'] & 80
    assert fault['driver']['spi_errors'] == fault['driver']['timeouts'] == 0
    fault_rows, _, _ = capture('fw8_mag_fault_20261019.txt')
    degraded = [row for row in fault_rows if row[17] == 0]
    assert len(degraded)>200 and all(row[10] == 1 and row[14] == row[16] == 0 for row in degraded)
    final = healthy(snapshot('fw8_final_state'))
    flash = (LOGS/'fw8_final_flash.bin').read_bytes()
    upper=verified=0
    image = ROOT/'MDK-ARM/MPU6000/MPU6000.hex'
    for line in image.read_text().splitlines():
        record=bytes.fromhex(line[1:])
        assert line.startswith(':') and sum(record)&255 == 0
        count,address,kind=record[0],int.from_bytes(record[1:3],'big'),record[3]
        payload=record[4:4+count]
        if kind == 4:
            upper=int.from_bytes(payload,'big')<<16
        elif kind == 0:
            offset=upper+address-0x08000000
            assert offset>=0 and offset+count<=len(flash)
            assert flash[offset:offset+count] == payload
            verified+=count
        else:
            assert kind in (1,5)
    assert verified>25000
    compatibility=json.loads((LOGS/'fw8_compatibility.json').read_text())
    assert all(item['exit']==0 for item in compatibility.values())
    tests=(LOGS/'fw8_tests.log').read_text(encoding='utf-8-sig')
    assert 'Ran 66 tests' in tests and 'OK' in tests and 'skipped' not in tests
    calibration=json.loads((LOGS/'fw8_mag_calibration_fit.json').read_text())
    metadata=json.loads((LOGS/'fw8_final_60s_20261019.txt.json').read_text(encoding='utf-8-sig'))
    report=dict(firmware='FW8 FireWater18 AHRS9',tests=66,steady_A=a,steady_B=b,
                measured_rates_hz=rates,uart_records=len(rows),uart_hz=len(rows)/metadata['elapsed_seconds'],
                capture_seconds=metadata['elapsed_seconds'],angles_deg=angles,
                capture_boundary_fragments=fragments,magnetic_calibration=calibration,
                magnetic_driver_software_fault=fault,mag_fault_degraded_rows=len(degraded),
                warm_resets=warm,final_state=final,flash_verified_bytes=verified,
                hex_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),compatibility_builds=compatibility,
                limitations='Magnetic north, declination=0; static local validation and software fault injection, not surveyed heading accuracy, thermal calibration or flight acceptance.')
    (LOGS/'fw8_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ['measured_rates_hz','uart_records','uart_hz','angles_deg','hex_sha256']},indent=2))


if __name__ == '__main__':
    main()
