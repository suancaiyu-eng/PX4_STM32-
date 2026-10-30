"""Restore production F1 sources, safe PWM init, compiler and memory layout.

Run after CubeMX generation. Does not download or run target firmware.
"""
from pathlib import Path
import re
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
    set_value(common, "Cpu", "IRAM(0x20000000-0x20004FFF) IROM(0x8000000-0x800F7FF) CLOCK(64000000) CPUTYPE(\"Cortex-M3\") TZ")
    arm = target.find("./TargetOption/TargetArmAds")
    set_value(arm.find("Cads"), "uC99", "1")
    control = arm.find("./Cads/VariousControls")
    include = control.find("IncludePath")
    paths = include.text.split(";")
    if "../../../firmware/rc" not in paths:
        paths.append("../../../firmware/rc")
    if "../../../firmware/common" not in paths:
        paths.append("../../../firmware/common")
    include.text = ";".join(paths)
    # Reserve the final two 1 KiB Flash pages for the monotonic boot journal.
    memories = arm.find("./ArmAdsMisc/OnChipMemories")
    for name, address, size in (("IROM", "0x08000000", "0x0000F800"),
                                ("OCR_RVCT4", "0x08000000", "0x0000F800"),
                                ("IRAM", "0x20000000", "0x00005000"),
                                ("OCR_RVCT9", "0x20000000", "0x00005000")):
        set_value(memories.find(name), "StartAddress", address)
        set_value(memories.find(name), "Size", size)
    misc = arm.find("ArmAdsMisc")
    for key in ("Ir2Chk", "Im2Chk", "hadIROM2"):
        set_value(misc, key, "0")

    groups = target.find("Groups")
    source_groups = {
        "Application/User/Core": ("../Core/Src/usart.c", "../Core/Src/r9ds.c",
                                  "../Core/Src/r9ds_telem.c", "../Core/Src/r9ds_calibration.c",
                                  "../Core/Src/tim.c", "../Core/Src/main_pwm.c",
                                  "../Core/Src/main_pwm_port.c", "../Core/Src/actuator_io.c",
                                  "../Core/Src/io_boot_id.c", "../Core/Src/io_boot_id_port.c"),
        "Application/R9DS/Protocol": ("../../../firmware/rc/sbus.c",),
        "Application/BoardLink": ("../../../firmware/common/board_link_protocol.c",
                                  "../../../firmware/common/board_link.c",
                                  "../../../firmware/common/board_link_app.c",
                                  "../../../firmware/common/actuator_protocol.c"),
        "Drivers/STM32F1xx_HAL_Driver": ("../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c",
                                      "../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c",
                                      "../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c"),
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

    # Production has one target and no bench sources, macros or include paths.
    targets = tree.find("Targets")
    for other in list(targets):
        if other is not target: targets.remove(other)
    target.find("TargetName").text = "F1"
    # CubeMX may restore generated early MX_TIM calls.
    # Keep pin handover and timer init inside MainPwm_Init (safe-low first).
    main_file = ROOT / "Core/Src/main.c"
    main_text = main_file.read_text(encoding="utf-8")
    main_text = re.sub(r"^\s*MX_TIM[234]_Init\(\);[^\n]*\n", "\n", main_text, flags=re.M)
    main_file.write_text(main_text, encoding="utf-8")
    conf = ROOT / "Core/Inc/stm32f1xx_hal_conf.h"
    conf.write_text(conf.read_text(encoding="utf-8").replace(
        "/*#define HAL_TIM_MODULE_ENABLED   */", "#define HAL_TIM_MODULE_ENABLED"), encoding="utf-8")
    ET.indent(tree, space="  ")
    tree.write(PROJECT, encoding="utf-8", xml_declaration=True)
    print("Production F1 only: four MAIN outputs, 62 KiB app + 2 KiB boot journal / 20 KiB RAM.")


if __name__ == "__main__":
    prepare()
