from pathlib import Path
import hashlib
import json
import re
import statistics
import struct

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / 'Logs'
pattern = re.compile(r'^mpu6000:(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)$')
rows, other = [], []
for line in (LOGS / 'fw6_60s_20261019.txt').read_text(encoding='utf-8-sig').splitlines():
    match = pattern.fullmatch(line.strip())
    if match:
        row = tuple(map(int, match.groups()))
        assert all(-32768 <= x <= 32767 for x in row)
        rows.append(row)
    elif line.strip():
        other.append(line)

status_names = ['drdy_count', 'samples', 'busy_drops', 'queue_drops', 'invalid_samples',
                'spi_errors', 'timeouts', 'last_sample_us', 'max_transfer_us']
tx_names = ['frames_started', 'frames_completed', 'frames_dropped', 'uart_errors',
            'tx_timeouts', 'start_errors', 'aborts', 'abort_timeouts', 'last_hal_error', 'status_frames']


def decode_status(name):
    # Arm Compiler 5 enums are byte-sized in this build. Map: g_imu_status at
    # 0x20000560, 44 bytes; g_telemetry_status immediately after it, 44 bytes.
    data = (LOGS / name).read_bytes()
    assert len(data) == 88
    result = dict(zip(['state', 'error', 'id', 'reg', 'wr', 'rd'], struct.unpack_from('<6B', data)))
    result.update(zip(status_names, struct.unpack_from('<9I', data, 8)))
    result['telemetry'] = dict(zip(tx_names, struct.unpack_from('<10I', data, 44)))
    result['telemetry'].update(zip(['busy', 'faulted'], struct.unpack_from('<2B', data, 84)))
    return result


start = decode_status('fw6_state_A.bin')
end = decode_status('fw6_state_B.bin')
elapsed_us = (end['last_sample_us'] - start['last_sample_us']) & 0xffffffff
meta = json.loads((LOGS / 'fw6_60s_20261019.txt.json').read_text(encoding='utf-8-sig'))
report = {
    'records': len(rows), 'capture_seconds': meta['elapsed_seconds'],
    'output_hz': len(rows) / meta['elapsed_seconds'], 'other_lines': other,
    'state_A': start, 'state_B': end,
    'sample_rate_hz': ((end['samples'] - start['samples']) & 0xffffffff) * 1e6 / elapsed_us,
    'uart_rate_hz': ((end['telemetry']['frames_completed'] - start['telemetry']['frames_completed']) & 0xffffffff) * 1e6 / elapsed_us,
    'steady_frames_dropped_delta': end['telemetry']['frames_dropped'] - start['telemetry']['frames_dropped'],
    'channels': {name: {'min': min(r[i] for r in rows), 'max': max(r[i] for r in rows),
                        'mean': round(statistics.mean(r[i] for r in rows), 3)}
                 for i, name in enumerate(['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp'])},
    'hex_sha256': hashlib.sha256((ROOT / 'MDK-ARM/MPU6000/MPU6000.hex').read_bytes()).hexdigest(),
}
assert len(rows) > 5900 and not other
assert end['state'] == 6 and end['error'] == 0 and end['id'] == 0x68
assert all(end[k] == 0 for k in ['busy_drops', 'queue_drops', 'invalid_samples', 'spi_errors', 'timeouts'])
assert all(end['telemetry'][k] == 0 for k in ['uart_errors', 'tx_timeouts', 'start_errors', 'aborts', 'abort_timeouts', 'faulted'])
# Initial invalid/throttled diagnostic slots count as dropped frames; require
# no additional drop in the steady interval instead of claiming boot drops=0.
assert report['steady_frames_dropped_delta'] == 0
assert 995 < report['sample_rate_hz'] < 1005
assert 99 < report['output_hz'] < 101
fault = decode_status('fw6_state_fault.bin')
assert fault['state'] == 7 and fault['error'] == 7 and fault['timeouts'] == 1
fault_lines = (LOGS / 'fw6_drdy_loss_20261019.txt').read_text(encoding='utf-8-sig').splitlines()
fault_start = next(i for i, line in enumerate(fault_lines) if line.startswith('#IMU state=7'))
assert not any(line.startswith('mpu6000:') for line in fault_lines[fault_start:])
warm_resets = []
for index in range(1, 4):
    text = (LOGS / f'fw6_warm_reset_0{index}.txt').read_text(encoding='utf-8-sig')
    offset = text.rfind('#BOOT MPU6000 FW6')
    assert offset >= 0
    post_boot = text[offset:]
    assert '#IMU state=7' not in post_boot
    count = sum(bool(pattern.fullmatch(line.strip())) for line in post_boot.splitlines())
    assert count > 100
    warm_resets.append({'round': index, 'valid_records_after_boot': count})
final = decode_status('fw6_state_final.bin')
assert final['state'] == 6 and final['error'] == 0 and final['id'] == 0x68
assert final['timeouts'] == 0 and final['spi_errors'] == 0


def live_capture(name):
    lines = (LOGS / name).read_text(encoding='utf-8-sig').splitlines()
    values, boundary_fragments = [], []
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        match = pattern.fullmatch(line.strip())
        if match:
            row = tuple(map(int, match.groups()))
            assert all(-32768 <= value <= 32767 for value in row)
            values.append(row)
        else:
            # Opening/closing a running UART can cut the first/last line.
            # Keep this evidence, and reject malformed interior lines.
            assert index in (0, len(lines) - 1), (name, index, line)
            assert re.fullmatch(r'[mpu6000:\d,\-]+', line.strip()), line
            boundary_fragments.append({'line_index': index, 'text': line})
    assert len(values) > 450
    metadata = json.loads((LOGS / (name + '.json')).read_text(encoding='utf-8-sig'))
    return values, {'file': name, 'records': len(values),
                    'capture_seconds': metadata['elapsed_seconds'],
                    'boundary_fragments': boundary_fragments}


def channel_statistics(values):
    return {name: {'min': min(row[i] for row in values),
                   'max': max(row[i] for row in values),
                   'mean': round(statistics.mean(row[i] for row in values), 3)}
            for i, name in enumerate(['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp'])}


motion_rows, motion_capture = live_capture('fw6_motion_followup_20261019.txt')
settled_rows, settled_capture = live_capture('fw6_motion_settled_20261019.txt')
motion_channels = channel_statistics(motion_rows)
assert all(motion_channels[name]['max'] - motion_channels[name]['min'] > 200
           for name in ['ax', 'ay', 'az', 'gx', 'gy', 'gz'])
settled_channels = channel_statistics(settled_rows[-100:])
assert all(settled_channels[name]['max'] - settled_channels[name]['min'] < 20
           for name in ['gx', 'gy', 'gz'])
gravity_counts = sum(settled_channels[name]['mean'] ** 2
                     for name in ['ax', 'ay', 'az']) ** 0.5
assert 2000 < gravity_counts < 2100
motion_state = decode_status('fw6_state_motion.bin')
assert motion_state['state'] == 6 and motion_state['error'] == 0 and motion_state['id'] == 0x68
assert all(motion_state[key] == 0 for key in
           ['busy_drops', 'queue_drops', 'invalid_samples', 'spi_errors', 'timeouts'])
assert all(motion_state['telemetry'][key] == 0 for key in
           ['uart_errors', 'tx_timeouts', 'start_errors', 'aborts', 'abort_timeouts', 'faulted'])
assert motion_state['samples'] > final['samples']
assert motion_state['telemetry']['frames_dropped'] == end['telemetry']['frames_dropped']
report.update(fault_injection=fault, warm_resets=warm_resets, final_state=final,
              motion_test={'confirmed': True, 'motion_capture': motion_capture,
                           'motion_channels': motion_channels,
                           'followup_capture': settled_capture,
                           'settled_last_100_records': settled_channels,
                           'settled_gravity_counts': round(gravity_counts, 3),
                           'post_motion_state': motion_state})
(LOGS / 'fw6_validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2))
