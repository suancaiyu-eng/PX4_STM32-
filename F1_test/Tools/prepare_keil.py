"""Validate/restore this standalone scope lab only. Never reads another project."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MDK-ARM/F1_test.uvprojx"


def prepare():
    tree = ET.parse(PROJECT)
    expected = {"F1_PWM_SWEEP": 0, "F1_PWM_TEST": 1}
    targets = tree.findall("./Targets/Target")
    assert {t.findtext("TargetName") for t in targets} == set(expected)
    for target in targets:
        name = target.findtext("TargetName")
        target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define").text = (
            "USE_HAL_DRIVER,STM32F103xB,F1_TEST_WATCH_MODE=" + str(expected[name]))
        for file in target.findall("./Groups/Group/Files/File/FilePath"):
            path = (PROJECT.parent / file.text.replace("\\", "/")).resolve()
            assert path.is_relative_to(ROOT.resolve()) and path.is_file(), str(path)
        includes = target.findtext("./TargetOption/TargetArmAds/Cads/VariousControls/IncludePath")
        for include in includes.split(";"):
            path = (PROJECT.parent / include.replace("\\", "/")).resolve()
            assert path.is_relative_to(ROOT.resolve()) and path.is_dir(), str(path)
    ET.indent(tree, space="  ")
    tree.write(PROJECT, encoding="utf-8", xml_declaration=True)
    print("Standalone lab validated: all compiled sources and includes are local.")


if __name__ == "__main__":
    prepare()
