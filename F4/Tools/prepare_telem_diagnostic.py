"""Build-only diagnostic project; the default gyro project is left untouched.

The diagnostic image emits boot/SPI-probe text, then either seven real raw IMU
channels or a rate-limited #IMU fault line. It never enables actuator outputs.
Flash only after identifying the connected MCU as STM32F427, not the F1 I/O MCU.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
tree = ET.parse(root / "MDK-ARM/MPU6000.uvprojx")
targets = tree.find("Targets")
for target in list(targets):
    if target.findtext("TargetName") != "MPU6000":
        targets.remove(target)
target = targets.find("Target")
common = target.find("./TargetOption/TargetCommonOption")
for key, value in (("OutputDirectory", "TELEM_DIAG\\"),
                   ("OutputName", "F4_TELEM_DIAG"),
                   ("ListingPath", "./TELEM_DIAG/")):
    common.find(key).text = value
define = target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define")
overrides = {
    "IMU_TELEMETRY_OUTPUT_FORMAT": "1",
    "IMU_TELEMETRY_RC_ONLY": "0",
    "FLIGHT_IMU_FIREWATER_ATTITUDE": "0",
    "ACTUATOR_CLIENT_TEST_ENABLED": "0",
    "BARO_TELEMETRY_ENABLED": "0",
}
definitions = [item for item in define.text.split(",")
               if item.strip().split("=", 1)[0] not in overrides]
define.text = ",".join(definitions + [f"{key}={value}" for key, value in overrides.items()])
output = root / "MDK-ARM/F4_TELEM_DIAG.uvprojx"
tree.write(output, encoding="utf-8", xml_declaration=True)
print(output)
