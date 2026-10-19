"""Validate final FW7 hardware evidence. No writes to the connected board."""
from pathlib import Path
import hashlib
import json
import math
import re
import statistics
import struct

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / 'Logs'


def capture(name, boot=False):
    text = (LOGS / name).read_text(encoding='utf-8-sig')
    if boot:
        index = text.rfind('#BOOT MPU6000 FW7 ')
        assert index >= 0 and 'FireWater11 AHRS6' in text[index:index+100]
        text = text[index:]
    lines = text.splitlines()
    rows, diagnostics, fragments = [], [], []
    for index, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue
        if line.startswith('#'):
            diagnostics.append(line)
            continue
        if not line.startswith('mpu6000:'):
            assert not boot and index in (0, len(lines)-1), (name, index, line)
            fragments.append(line)
            continue
        parts = line[8:].split(',')
        if len(parts) != 11:
            assert not boot and index in (0, len(lines)-1), (name, index, line)
            fragments.append(line)
            continue
        assert all(re.fullmatch(r'-?\d+', value) for value in parts[:7])
        values = [int(value) for value in parts[:7]]
        assert all(-32768 <= value <= 32767 for value in values)
        angles = [float(value) for value in parts[7:10]]
        assert all(math.isfinite(value) and abs(value) <= 180 for value in angles)
        assert parts[10] in ('0', '1')
        valid = int(parts[10])
        if not valid:
            assert angles == [0, 0, 0]
        rows.append(values + angles + [valid])
    return rows, diagnostics, fragments


def snapshot(tag):
    metadata = json.loads((LOGS / (tag+'.json')).read_text(encoding='utf-8-sig'))
    data = (LOGS / (tag+'.bin')).read_bytes()
    def offset(name):
        return metadata['symbols'][name]['address'] - metadata['base']
    driver_offset = offset('g_imu_status')
    state, error, identity, *_ = struct.unpack_from('<6B', data, driver_offset)
    driver = dict(zip(['drdy', 'samples', 'busy_drops', 'queue_drops', 'invalid_samples',
                       'spi_errors', 'timeouts', 'timestamp', 'max_dma_us'],
                      struct.unpack_from('<9I', data, driver_offset+8)))
    driver.update(state=state, error=error, id=identity)
    tx_offset = offset('g_telemetry_status')
    uart = dict(zip(['started', 'completed', 'dropped', 'errors', 'timeouts', 'start_errors',
                    'aborts', 'abort_timeouts', 'hal_error', 'status_frames'],
                   struct.unpack_from('<10I', data, tx_offset)))
    uart['busy'], uart['faulted'] = struct.unpack_from('<2B', data, tx_offset+40)
    uart['attitude_frames'], uart['attitude_status_frames'] = struct.unpack_from('<2I', data, tx_offset+44)
    pose_offset = offset('g_attitude')
    pose = dict(zip(['timestamp', 'sequence', 'state', 'error', 'flags'],
                   struct.unpack_from('<5I', data, pose_offset)))
    pose['q'] = struct.unpack_from('<4f', data, pose_offset+20)
    pose['euler_deg'] = [math.degrees(value) for value in struct.unpack_from('<3f', data, pose_offset+36)]
    pose['bias_rad_s'] = struct.unpack_from('<3f', data, pose_offset+48)
    pose.update(zip(['inputs', 'predictions', 'corrections', 'publications', 'calibration_samples',
                     'calibration_restarts', 'accel_rejections', 'timing_faults', 'gap_faults'],
                    struct.unpack_from('<9I', data, pose_offset+84)))
    pose['max_update_us'], = struct.unpack('<I', (LOGS / (tag+'_max.bin')).read_bytes())
    return dict(driver=driver, uart=uart, attitude=pose)


def healthy(state):
    driver, uart, pose = state['driver'], state['uart'], state['attitude']
    assert (driver['state'], driver['error'], driver['id']) == (6, 0, 0x68)
    assert all(driver[key] == 0 for key in ['busy_drops', 'queue_drops', 'invalid_samples', 'spi_errors', 'timeouts'])
    assert all(uart[key] == 0 for key in ['errors', 'timeouts', 'start_errors', 'aborts', 'abort_timeouts', 'faulted'])
    assert (pose['state'], pose['error']) == (2, 0) and pose['flags'] & 7 == 7
    assert pose['calibration_samples'] >= 2000
    assert pose['timing_faults'] == pose['gap_faults'] == 0
    assert abs(sum(value*value for value in pose['q']) - 1) < 0.00001
    assert pose['max_update_us'] < 100
    return state


def main():
    a, b = healthy(snapshot('fw7_steady_A')), healthy(snapshot('fw7_steady_B'))
    elapsed = ((b['attitude']['timestamp'] - a['attitude']['timestamp']) & 0xffffffff)*1e-6
    rates = {key: ((b['attitude'][key] - a['attitude'][key]) & 0xffffffff)/elapsed
             for key in ['predictions', 'corrections', 'publications']}
    assert 995 < rates['predictions'] < 1005
    assert 199 < rates['publications'] < 201 and 199 < rates['corrections'] < 201
    assert a['uart']['dropped'] == b['uart']['dropped']
    rows, diagnostics, fragments = capture('fw7_60s_20261019.txt')
    assert len(rows) > 5900 and not diagnostics and all(row[10] == 1 for row in rows)
    channels = {name: dict(min=min(row[i] for row in rows), max=max(row[i] for row in rows),
                           mean=statistics.mean(row[i] for row in rows),
                           stdev=statistics.pstdev(row[i] for row in rows))
                for i, name in enumerate(['ax','ay','az','gx','gy','gz','temp','roll','pitch','yaw','valid'])}
    assert channels['roll']['stdev'] < 0.2 and channels['pitch']['stdev'] < 0.2
    boot_rows, boot_diagnostics, _ = capture('fw7_boot_firewater11_20261019.txt', boot=True)
    assert any(row[10] == 0 for row in boot_rows) and any(row[10] == 1 for row in boot_rows)
    assert not any('#IMU state=7' in line or '#ATT state=3' in line for line in boot_diagnostics)
    warm = []
    for index in range(1, 3):
        values, messages, _ = capture(f'fw7_warm_reset_0{index}.txt', boot=True)
        assert sum(row[10] == 1 for row in values) > 100
        assert any('#ATT state=1' in line for line in messages)
        assert not any('#IMU state=7' in line or '#ATT state=3' in line for line in messages)
        warm.append({'round': index, 'valid_records': sum(row[10] == 1 for row in values)})
    fault = snapshot('fw7_fault_state')
    assert (fault['driver']['state'], fault['driver']['error']) == (7, 7)
    assert fault['attitude']['state'] == 3 and fault['attitude']['error'] == 4
    assert fault['attitude']['flags'] & 1 == 0
    text = (LOGS/'fw7_drdy_loss_20261019.txt').read_text(encoding='utf-8-sig')
    point = text.index('#IMU state=7')
    assert 'mpu6000:' not in text[point:]
    final = healthy(snapshot('fw7_final_state'))
    # Compare every data byte in the final Intel HEX to the actual target Flash.
    flash = (LOGS/'fw7_final_flash.bin').read_bytes()
    upper, verified_bytes = 0, 0
    for line in (ROOT/'MDK-ARM/MPU6000/MPU6000.hex').read_text().splitlines():
        record = bytes.fromhex(line[1:])
        assert line.startswith(':') and sum(record) & 0xff == 0
        count, address, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
        payload = record[4:4+count]
        if kind == 4:
            upper = int.from_bytes(payload, 'big') << 16
        elif kind == 0:
            offset = upper + address - 0x08000000
            assert offset >= 0 and offset + count <= len(flash)
            assert flash[offset:offset+count] == payload
            verified_bytes += count
        else:
            assert kind in (1, 5), kind
    assert verified_bytes > 20000
    compatibility = json.loads((LOGS/'fw7_compatibility.json').read_text(encoding='utf-8'))
    assert all(item['exit'] == 0 for item in compatibility.values())
    metadata = json.loads((LOGS/'fw7_60s_20261019.txt.json').read_text(encoding='utf-8-sig'))
    report = {'firmware': 'FW7 FireWater11 AHRS6', 'steady_A': a, 'steady_B': b,
              'measured_rates_hz': rates, 'uart_records': len(rows),
              'capture_seconds': metadata['elapsed_seconds'],
              'uart_hz': len(rows)/metadata['elapsed_seconds'], 'channels': channels,
              'capture_boundary_fragments': fragments,
              'flash_verified_bytes': verified_bytes, 'compatibility_builds': compatibility,
              'warm_resets': warm, 'fault_injection': fault, 'final_state': final,
              'hex_sha256': hashlib.sha256((ROOT/'MDK-ARM/MPU6000/MPU6000.hex').read_bytes()).hexdigest(),
              'motion_validation': 'Actual C checked against analytic +/-90 degree rotations and coupled motion; no physical rotation requested for this migration run',
              'limitations': 'Relative yaw only; startup stationary gyro bias, not full accelerometer calibration or flight acceptance'}
    (LOGS/'fw7_validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: report[key] for key in ['measured_rates_hz','uart_records','uart_hz','hex_sha256']}, indent=2))


if __name__ == '__main__':
    main()
