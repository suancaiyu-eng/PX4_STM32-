"""Finish physical two-way testing when the existing ST-Link is moved to F4.

Only Device ID 0x419 is programmed, after backing up the complete detected flash.
The F1 image must already be running. Normal board power must power both MCUs.
No software loopback or injected replies are counted as successful communication.
"""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import time

from board_link_probe import CLI, run as read_live

ROOT = Path(__file__).resolve().parents[3]
F4 = ROOT / "code_keil/F4"
LOGS = F4 / "Logs"
HEX = F4 / "MDK-ARM/MPU6000/MPU6000.hex"
LOGS.mkdir(exist_ok=True)


def cli(arguments, log, mode="HOTPLUG", timeout=120):
    command = [str(CLI), "-c", "port=SWD", "sn=35FF72063448303417211657", "freq=1000", "mode=" + mode, *arguments]
    completed = subprocess.run(command, capture_output=True, timeout=timeout,
                               creationflags=subprocess.CREATE_NO_WINDOW)
    text = completed.stdout.decode("utf-8", errors="replace") + completed.stderr.decode("utf-8", errors="replace")
    log.write_text(text, encoding="utf-8")
    return completed.returncode, text


def audit(result):
    first = result["snapshots"][0]
    last = result["snapshots"][-1]
    failures = []
    if last["elapsed_seconds"] - first["elapsed_seconds"] < 59:
        failures.append("Less than 59 seconds between running snapshots")
    a, b = first["g_board_link_test"], last["g_board_link_test"]
    if b["peer_online"] != 1 or b["peer_node"] != 1:
        failures.append("F1 peer not online")
    for name in ("replies_matched", "requests_answered"):
        if b[name] - a[name] < 100:
            failures.append("Insufficient real two-way messages: " + name)
    for name in ("reply_timeouts", "bad_replies"):
        if b[name] != a[name]:
            failures.append("New " + name)
    a, b = first["g_board_link_status"], last["g_board_link_status"]
    for name in ("crc_errors", "header_errors", "uart_errors", "rx_overflows", "tx_full", "queue_drops"):
        if b[name] != a[name]:
            failures.append("New " + name)
    return failures


def main(wait_seconds):
    deadline = time.monotonic() + wait_seconds
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    previous_state = None
    programmed = False
    print("Waiting for the existing ST-Link to access F4 (device ID 0x419). Both MCUs require board power.", flush=True)
    while time.monotonic() < deadline:
        code, text = cli(["-r32", "0xE0042000", "4"], LOGS / "board_link_wait_probe.log", timeout=20)
        match = re.search(r"Device ID\s*:\s*0x([0-9a-fA-F]+)", text)
        device = int(match[1], 16) if code == 0 and match else None
        state = "F1 attached; F4 download port still inaccessible" if device == 0x410 else (
            "F4 attached" if device == 0x419 else "No supported target currently accessible")
        if state != previous_state:
            print(state, flush=True)
            previous_state = state
        if device != 0x419:
            time.sleep(min(10, max(0, deadline - time.monotonic())))
            continue
        if not programmed:
            size_dump = LOGS / ("f4_flash_size_" + stamp + ".bin")
            code, _ = cli(["-u", "0x1FFF7A20", "4", str(size_dump)], LOGS / ("f4_flash_size_" + stamp + ".log"))
            if code != 0 or not size_dump.is_file():
                raise RuntimeError("Cannot read F4 flash size; no programming performed")
            flash_kib = struct.unpack_from("<H", size_dump.read_bytes(), 2)[0]
            if flash_kib not in (1024, 2048):
                raise RuntimeError("Unexpected F4 flash size; no programming performed")
            backup = LOGS / ("F4_before_board_link_" + stamp + ".bin")
            code, _ = cli(["-u", "0x08000000", str(flash_kib * 1024), str(backup)],
                          LOGS / ("f4_backup_" + stamp + ".log"))
            if code != 0 or not backup.is_file() or backup.stat().st_size != flash_kib * 1024:
                raise RuntimeError("F4 complete flash backup failed; no programming performed")
            evidence = {"firmware": str(HEX), "sha256": hashlib.sha256(HEX.read_bytes()).hexdigest(),
                        "backup": str(backup), "flash_kib": flash_kib, "device_id": device,
                        "backup_sha256": hashlib.sha256(backup.read_bytes()).hexdigest()}
            (LOGS / "board_link_programming.json").write_text(json.dumps(evidence, indent=2), encoding="utf-8")
            code, text = cli(["-w", str(HEX), "-v", "-rst", "-run"], LOGS / "board_link_programming.log", mode="NORMAL")
            if code != 0 or "Download verified successfully" not in text:
                raise RuntimeError("F4 programming/verification failed; see board_link_programming.log")
            programmed = True
            print("F4 firmware programmed and verified; waiting for real F1 peer replies.", flush=True)
            time.sleep(3)
        result = read_live(F4, "f4_board_link_discovery", 0, 10)
        if result["snapshots"][-1]["g_board_link_test"]["peer_online"] == 1:
            print("Real F1 peer replies detected. Starting 60-second two-way verification.", flush=True)
            result = read_live(F4, "f4_board_link_60s", 62, 10)
            failures = audit(result)
            summary = {"passed": not failures, "failures": failures,
                       "evidence": str(LOGS / "f4_board_link_60s.json"),
                       "verification": "Physical F4 USART6 <-> F1 USART2, CRC checked and request payloads matched"}
            (LOGS / "board_link_result.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            if failures:
                raise RuntimeError("Physical verification failed: " + "; ".join(failures))
            print("PASS: physical F1/F4 bidirectional communication verified for 60 seconds.", flush=True)
            return
        time.sleep(min(10, max(0, deadline - time.monotonic())))
    incomplete = {"passed": False, "incomplete": True, "f4_programmed": programmed,
                  "reason": "F4 download access or a powered/running F1 peer was not available before the wait deadline"}
    (LOGS / "board_link_result.json").write_text(json.dumps(incomplete, indent=2), encoding="utf-8")
    print("INCOMPLETE: physical F1/F4 communication is not verified. See board_link_result.json.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wait-seconds", type=float, default=900)
    args = parser.parse_args()
    if args.wait_seconds <= 0:
        parser.error("wait-seconds must be positive")
    main(args.wait_seconds)
