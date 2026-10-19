"""Build both retained wire formats without modifying/flashing the main target."""
from pathlib import Path
import json
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
UV4 = r'D:\Users\21144\AppData\Local\Keil_v5\UV4\UV4.exe'
results = {}
for name, define in [('binary', 'IMU_TELEMETRY_OUTPUT_FORMAT=0'),
                     ('raw7', 'FLIGHT_IMU_FIREWATER_ATTITUDE=0')]:
    temporary = ROOT / 'MDK-ARM' / f'FW8_verify_{name}.uvprojx'
    assert not temporary.exists()
    tree = ET.parse(ROOT / 'MDK-ARM/MPU6000.uvprojx')
    target = tree.find('./Targets/Target')
    common = target.find('./TargetOption/TargetCommonOption')
    common.find('OutputDirectory').text = f'..\\Logs\\fw8_compat_{name}\\'
    common.find('OutputName').text = f'MPU6000_{name}'
    common.find('ListingPath').text = f'..\\Logs\\fw8_compat_{name}\\'
    definitions = target.find('./TargetOption/TargetArmAds/Cads/VariousControls/Define')
    definitions.text += ',' + define
    log = ROOT / 'Logs' / f'fw8_compat_{name}.log'
    try:
        tree.write(temporary, encoding='utf-8', xml_declaration=True)
        completed = subprocess.run([UV4, '-b', str(temporary), '-o', str(log)],
                                   creationflags=subprocess.CREATE_NO_WINDOW,
                                   timeout=120, check=False)
        content = log.read_text(errors='replace')
        assert completed.returncode == 0 and '0 Error(s), 0 Warning(s)' in content, content
        results[name] = dict(exit=completed.returncode, log=log.name,
                             hex=str(ROOT/'Logs'/f'fw8_compat_{name}'/f'MPU6000_{name}.hex'))
        print(name, '0 errors / 0 warnings', flush=True)
    finally:
        temporary.unlink(missing_ok=True)
(ROOT/'Logs/fw8_compatibility.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
