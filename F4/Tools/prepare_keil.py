"""Restore application sources and DMA-safe linker settings after CubeMX generation.

Run from any directory: python Tools/prepare_keil.py
Does not change programmer/debugger settings or download anything.
"""
from pathlib import Path
import copy
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MDK-ARM/MPU6000.uvprojx"


def prepare():
    tree = ET.parse(PROJECT)
    target = tree.find("./Targets/Target")
    assert target is not None

    def set_value(parent, name, value):
        node = parent.find(name)
        if node is None:
            node = ET.SubElement(parent, name)
        node.text = value

    # Installed compiler: Arm Compiler 5.06 update 7. C99 is used throughout.
    set_value(target, "pCCUsed", "5060960::V5.06 update 7 (build 960)::ARM_Compiler_5.06u7")
    set_value(target, "uAC6", "0")
    arm = target.find("./TargetOption/TargetArmAds")
    ld = arm.find("LDads")
    for key, value in {"umfTarg": "0", "useFile": "1", "ScatterFile": ".\\MPU6000.sct"}.items():
        set_value(ld, key, value)
    common = target.find("./TargetOption/TargetCommonOption")
    common.find("Cpu").text = common.find("Cpu").text.replace("CLOCK(25000000)", "CLOCK(180000000)")
    define = arm.find("./Cads/VariousControls/Define")
    definitions = [s for s in define.text.split(",") if not s.startswith("HSE_VALUE=")]
    define.text = ",".join(definitions + ["HSE_VALUE=24000000U"])
    arm.find("./Cads/uC99").text = "1"
    includes = arm.find("./Cads/VariousControls/IncludePath")
    paths = includes.text.split(";")
    if "../../../firmware/common" not in paths:
        paths.append("../../../firmware/common")
    includes.text = ";".join(paths)
    memories = arm.find("./ArmAdsMisc/OnChipMemories")
    for name, address, size in [("IROM", "0x08000000", "0x00200000"),
                                ("OCR_RVCT4", "0x08000000", "0x00200000"),
                                ("IRAM", "0x20000000", "0x00030000"),
                                ("OCR_RVCT9", "0x20000000", "0x00030000")]:
        set_value(memories.find(name), "StartAddress", address)
        set_value(memories.find(name), "Size", size)
    for key in ("Ir2Chk", "Im2Chk", "hadIROM2"):
        set_value(arm.find("ArmAdsMisc"), key, "0")

    groups = target.find("Groups")
    group = next((g for g in groups if g.findtext("GroupName") == "Application/FlightIMU"), None)
    if group is None:
        group = ET.SubElement(groups, "Group")
        ET.SubElement(group, "GroupName").text = "Application/FlightIMU"
        ET.SubElement(group, "Files")
    files = group.find("Files")
    all_paths = {n.text.replace("\\", "/") for n in target.findall("./Groups/Group/Files/File/FilePath")}
    for name in ("mpu6000.c", "mpu6000_port.c", "imu_telemetry.c", "flight_imu_app.c", "attitude_estimator.c",
                 "lsm303d_mag.c", "lsm303d_mag_port.c", "ms56xx_baro.c", "ms56xx_baro_port.c",
                 "board_link_telem.c", "actuator_client.c"):
        path = "../Core/Src/" + name
        if path not in all_paths:
            file = ET.SubElement(files, "File")
            for key, value in {"FileName": name, "FileType": "1", "FilePath": path}.items():
                ET.SubElement(file, key).text = value
    group = next((g for g in groups if g.findtext("GroupName") == "Application/BoardLink"), None)
    if group is None:
        group = ET.SubElement(groups, "Group")
        ET.SubElement(group, "GroupName").text = "Application/BoardLink"
        ET.SubElement(group, "Files")
    for name in ("board_link_protocol.c", "board_link.c", "board_link_app.c", "actuator_protocol.c"):
        path = "../../../firmware/common/" + name
        if path not in all_paths:
            file = ET.SubElement(group.find("Files"), "File")
            for key, value in {"FileName": name, "FileType": "1", "FilePath": path}.items():
                ET.SubElement(file, key).text = value
    # Separate output trees prevent test-mailbox objects leaking into default.
    define.text = ",".join(x for x in define.text.split(",")
                           if not x.strip().startswith("ACTUATOR_CLIENT_TEST_ENABLED"))
    targets = tree.find("Targets")
    for old in list(targets):
        if old.findtext("TargetName") == "F4_ACTUATOR_TEST": targets.remove(old)
    test_target = copy.deepcopy(target)
    test_target.find("TargetName").text = "F4_ACTUATOR_TEST"
    test_common = test_target.find("./TargetOption/TargetCommonOption")
    set_value(test_common, "OutputDirectory", "F4_ACTUATOR_TEST\\")
    set_value(test_common, "OutputName", "F4_ACTUATOR_TEST")
    set_value(test_common, "ListingPath", "./F4_ACTUATOR_TEST/")
    test_define = test_target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define")
    test_define.text = define.text + ",ACTUATOR_CLIENT_TEST_ENABLED=1"
    targets.append(test_target)
    for old in list(targets):
        if old.findtext("TargetName") == "F4_BARO_TEST": targets.remove(old)
    baro_target = copy.deepcopy(target)
    baro_target.find("TargetName").text = "F4_BARO_TEST"
    baro_common = baro_target.find("./TargetOption/TargetCommonOption")
    set_value(baro_common, "OutputDirectory", "F4_BARO_TEST\\")
    set_value(baro_common, "OutputName", "F4_BARO_TEST")
    set_value(baro_common, "ListingPath", "./F4_BARO_TEST/")
    baro_define = baro_target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define")
    baro_define.text = define.text + ",BARO_TELEMETRY_ENABLED=1"
    targets.append(baro_target)
    ET.indent(tree, space="  ")
    tree.write(PROJECT, encoding="utf-8", xml_declaration=True)

    startup = ROOT / "MDK-ARM/startup_stm32f427xx.s"
    text = startup.read_text(encoding="utf-8")
    text, count = re.subn(r"(Stack_Size\s+EQU\s+)0x[0-9A-Fa-f]+", r"\g<1>0x1000", text)
    assert count == 1
    startup.write_text(text, encoding="utf-8")
    print("Keil sources, 24 MHz HSE define, 4 KiB stack and SRAM scatter file configured.")


if __name__ == "__main__":
    prepare()
