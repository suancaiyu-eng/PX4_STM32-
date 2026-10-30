"""Compile and execute the actual PWM policy and STM32 port using host stubs.

Software evidence only: does not measure the real pins or timer waveform.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
GCC = shutil.which("gcc") or r"D:\Users\21144\w64devkit\bin\gcc.exe"


class MainPwmTests(unittest.TestCase):
    def compile_run(self, name, sources, defines=(), stub=False):
        with tempfile.TemporaryDirectory(prefix="f1_pwm_") as temporary:
            binary = Path(temporary) / (name + ".exe")
            flags = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-O1"]
            if stub:
                flags += ["-I" + str(HERE / "pwm_stubs")]
            flags += ["-I" + str(ROOT / "Core/Inc")]
            subprocess.run([GCC, *flags, *defines, *map(str, sources), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

    def test_policy_default(self):
        self.compile_run("policy", [HERE / "test_main_pwm.c", ROOT / "Core/Src/main_pwm.c"])

    def test_policy_mailbox(self):
        self.compile_run("mailbox", [HERE / "test_main_pwm.c", ROOT / "Core/Src/main_pwm.c", ROOT / "Core/Src/main_pwm_bench.c"],
                         ["-DF1_TEST_WATCH_MODE=1"])

    def test_stm32_adapter(self):
        self.compile_run("port", [HERE / "test_main_pwm_port.c",
                                  ROOT / "Core/Src/main_pwm_port.c", ROOT / "Core/Src/tim.c"], stub=True)

    def test_scope_sweep(self):
        self.compile_run("sweep", [HERE / "test_main_pwm_sweep.c", ROOT / "Core/Src/main_pwm_sweep.c"],
                         ["-DF1_TEST_WATCH_MODE=0"])


if __name__ == "__main__":
    unittest.main()
