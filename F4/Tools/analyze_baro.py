"""Analyze actual bench captures; retain measured limitations, not just pass/fail."""
import csv
import json
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parents[1]
LOGS=ROOT/'Logs'
LABELS=['p5611_pa','p5607_pa','selected_pa','temperature_centi_c','relative_mm','filtered_mm',
        'flags','model','state','error','sequence','age_us','reference_samples','crc4']


def capture(name):
    path=LOGS/(name+'.txt')
    rows=[]; malformed=[]
    for index,line in enumerate(path.read_text().splitlines()):
        if not line.startswith('baro:'): continue
        try: row=list(map(int,line[5:].split(',')))
        except ValueError: malformed.append(index); continue
        if len(row)!=14: malformed.append(index); continue
        # Programming captures can begin with buffered output from the previous
        # image before reset. Retain only this board's selected model here.
        if row[7]==5611: rows.append(row)
    assert rows and not malformed,(name,malformed)
    good=[r for r in rows if r[6]&4]
    assert good and all(r[8]==2 and r[9]==0 and r[6]==7 for r in good)
    duration=json.loads(path.with_suffix('.txt.json').read_text(encoding='utf-8-sig'))['elapsed_seconds']
    # Text protocol has sample sequence and age, but no absolute timestamp.
    # Plot/CSV receive times are approximate at the measured console rate.
    period=duration/len(rows)
    with path.with_suffix('.csv').open('w',newline='') as out:
        writer=csv.writer(out); writer.writerow(['approx_receive_s',*LABELS])
        writer.writerows([[round(i*period,4),*r] for i,r in enumerate(rows)])
    baseline=statistics.mean(r[5]/1000 for r in good[:100])
    tail=statistics.mean(r[5]/1000 for r in good[-100:])
    result={'console_rows':len(rows),'capture_seconds':duration,'height_valid_rows':len(good),
        'console_hz':len(rows)/duration,'pressure_pa_range':[min(r[2] for r in good),max(r[2] for r in good)],
        'temperature_c_range':[min(r[3] for r in good)/100,max(r[3] for r in good)/100],
        'filtered_height_m_range':[min(r[5] for r in good)/1000,max(r[5] for r in good)/1000],
        'filtered_height_std_m':statistics.pstdev(r[5]/1000 for r in good),
        'first_5s_mean_m':baseline,'last_5s_mean_m':tail,'first_last_5s_change_m':tail-baseline,
        'peak_above_initial_mean_m':max(r[5] for r in good)/1000-baseline,
        'max_age_us':max(r[11] for r in good),
        'first_valid_sample_sequence':good[0][10], 'max_reference_samples':max(r[12] for r in good)}
    return result


def counters(name):
    snapshots=json.loads((LOGS/(name+'.json')).read_text())['snapshots']
    first,last=snapshots[0],snapshots[-1]
    dt=((last['baro']['last_sample_us']-first['baro']['last_sample_us'])&0xffffffff)/1e6
    assert dt>10
    deltas={}
    for section,fields in [('baro',['bus_errors']),('imu',['busy_drops','queue_drops','invalid_samples','spi_errors','timeouts']),
                           ('spi',['errors','deferred_drdy','lost_drdy']),('mag',['bus_errors','invalid_samples','overruns'])]:
        deltas[section]={field:last[section][field]-first[section][field] for field in fields}
    assert all(v==0 for section in deltas.values() for v in section.values()),deltas
    assert all(s['baro']['state']==2 and s['baro']['error']==0 and s['baro']['model']==5611
        and s['pressure_valid']==s['altitude_valid']==1 and s['imu']['state']==6 for s in snapshots)
    return {'window_seconds':dt,'baro_hz':(last['baro']['samples']-first['baro']['samples'])/dt,
        'imu_hz':(last['imu']['samples']-first['imu']['samples'])/dt,
        'mag_hz':(last['mag']['samples']-first['mag']['samples'])/dt,
        'counter_deltas':deltas,'max_baro_interval_us':last['baro']['max_interval_us'],
        'max_peer_spi_us':last['spi']['max_transfer_us'],
        'reference_pressure_pa':last['baro']['reference_pressure_pa'],
        'prom':[last['baro'][f'prom{i}'] for i in range(8)],
        'crc4':last['baro']['crc_actual'],'crc_expected':last['baro']['crc_expected'],
        'snapshot_note':'Live SWD reads are not atomic; rates are approximate long-window counter rates.'}


def main():
    result={'location':'Guilin, Guangxi, China; indoor (user supplied)',
        'selected_compensation_model':5611,'identification':'Measured pressure plausibility, not silicon model ID',
        'static':capture('baro_selected_static'),'static_counters':counters('baro_selected_static_swd'),
        'movement':capture('baro_lift'),'movement_counters':counters('baro_lift_swd')}
    fault=json.loads((LOGS/'baro_timeout_swd.json').read_text())['snapshots']
    assert all(s['baro']['state']==3 and s['baro']['error']==6 and s['sample']['flags']==0 and
        s['pressure_valid']==s['altitude_valid']==0 and s['imu']['state']==6 for s in fault)
    reset=[list(map(int,line[5:].split(','))) for line in (LOGS/'baro_timeout_reset.txt').read_text().splitlines()
           if line.startswith('baro:')]
    assert reset[-1][6:10]==[7,5611,2,0]
    result['timeout_injection']={'kind':'Software phase_us timestamp set to zero via SWD; not a physical bus fault',
        'latched_error':6,'valid_flags_cleared':True,'imu_kept_running':True,'reset_restored_valid_height':True}
    result['movement']['user_reported_approximate_height_m']=0.5
    result['movement']['user_reported_method']='Casual lift, holding edges; no obvious airflow'
    result['movement']['controlled_test']=False
    result['movement']['direction_verified']=False
    result['movement']['accuracy_verified']=False
    result['movement']['interpretation']='Exploratory trace only: no synchronized action markers or fixed height fixture; do not infer direction, scale or cause of transient.'
    result['limits']=['No flight altitude-hold controller or vertical inertial fusion is part of this driver task.',
        'The informal motion capture cannot validate direction or amplitude; no further motion measurement is scheduled.',
        'Raw barometric height contains drift and pressure/thermal disturbances; no absolute accuracy claimed.']
    (LOGS/'baro_hardware_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()
