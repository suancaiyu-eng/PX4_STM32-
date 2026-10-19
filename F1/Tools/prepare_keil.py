"""Register R9DS sources and the installed Arm Compiler 5.06u7 for Keil.

Run after CubeMX generation. Does not download or run target firmware.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MDK-ARM/F1.uvprojx"


def prepare():
    tree = ET.parse(PROJECT)
    target = tree.find("./Targets/Target")

    def set_value(parent, key, value):
        node = parent.find(key)
        if node is None:
            node = ET.SubElement(parent, key)
        node.text = value

    set_value(target, "pCCUsed", "5060960::V5.06 update 7 (build 960)::ARM_Compiler_5.06u7")
    set_value(target, "uAC6", "0")
    common = target.find("./TargetOption/TargetCommonOption")
    set_value(common, "Cpu", "IRAM(0x20000000-0x20004FFF) IROM(0x8000000-0x800FFFF) CLOCK(64000000) CPUTYPE(\"Cortex-M3\") TZ")
    arm = target.find("./TargetOption/TargetArmAds")
    set_value(arm.find("Cads"), "uC99", "1")
    control = arm.find("./Cads/VariousControls")
    include = control.find("IncludePath")
    paths = include.text.split(";")
    if "../../../firmware/rc" not in paths:
        paths.append("../../../firmware/rc")
    include.text = ";".join(paths)
    # Explicitly constrain the standalone F103C8 image to 64 KiB / 20 KiB.
    memories = arm.find("./ArmAdsMisc/OnChipMemories")
    for name, address, size in (("IROM", "0x08000000", "0x00010000"),
                                ("OCR_RVCT4", "0x08000000", "0x00010000"),
                                ("IRAM", "0x20000000", "0x00005000"),
                                ("OCR_RVCT9", "0x20000000", "0x00005000")):
        set_value(memories.find(name), "StartAddress", address)
        set_value(memories.find(name), "Size", size)
    misc = arm.find("ArmAdsMisc")
    for key in ("Ir2Chk", "Im2Chk", "hadIROM2"):
        set_value(misc, key, "0")

    groups = target.find("Groups")
    source_groups = {
        "Application/User/Core": ("../Core/Src/usart.c", "../Core/Src/r9ds.c"),
        "Application/R9DS/Protocol": ("../../../firmware/rc/sbus.c",),
        "Drivers/STM32F1xx_HAL_Driver": ("../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c",),
    }
    all_paths = {n.text.replace("\\", "/") for n in target.findall("./Groups/Group/Files/File/FilePath")}
    for group_name, sources in source_groups.items():
        group = next((g for g in groups if g.findtext("GroupName") == group_name), None)
        if group is None:
            group = ET.SubElement(groups, "Group")
            ET.SubElement(group, "GroupName").text = group_name
            ET.SubElement(group, "Files")
        for source in sources:
            if not (ROOT / "MDK-ARM" / source).is_file():
                raise FileNotFoundError(source)
            if source not in all_paths:
                file = ET.SubElement(group.find("Files"), "File")
                for key, value in (("FileName", Path(source).name), ("FileType", "1"), ("FilePath", source)):
                    ET.SubElement(file, key).text = value
                all_paths.add(source)

    ET.indent(tree, space="  ")
    tree.write(PROJECT, encoding="utf-8", xml_declaration=True)
    print("F1: R9DS sources, Arm Compiler 5.06u7, 64 KiB Flash / 20 KiB RAM configured.")


if __name__ == "__main__":
    prepare()
