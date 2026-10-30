"""Read live board-link evidence via the connected ST-Link; never programs/resets.

Run with --project D:/pixhawk2.4.8/code_keil/F4 for the F4 target.
Snapshots are read while the MCU runs, so counters may advance during a dump.
"""
import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import time

CLI = Path(r"D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe")
STATUS = ("magic node uptime_ms tx_packets tx_bytes rx_packets rx_bytes crc_errors header_errors "
          "uart_errors rx_overflows tx_full queue_drops wrong_address partial_timeouts last_rx_ms discarded_bytes").split()
TEST = ("magic node tests_sent replies_matched requests_answered reply_timeouts bad_replies "
        "unknown_commands last_round_trip_ms last_reply_ms peer_online last_sequence peer_node last_peer_length").split()


def run(project, tag, duration, interval):
    maps = list((project / "MDK-ARM").glob("*/*.map"))
    map_file = next(p for p in maps if p.name == ("F1.map" if project.name == "F1" else "MPU6000.map"))
    text = map_file.read_text(errors="replace")
    symbols = {}
    for name in ("g_board_link_status", "g_board_link_test"):
        match = re.search(r"(?m)^\s+" + name + r"\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)", text)
        if not match:
            raise RuntimeError("Missing map symbol " + name)
        symbols[name] = (int(match[1], 16), int(match[2]))
    base = min(address for address, _ in symbols.values())
    size = max(address + length for address, length in symbols.values()) - base
    if base < 0x20000000 or base + size > 0x20030000:
        raise RuntimeError("Map symbols outside normal SRAM")
    logs = project / "Logs"
    logs.mkdir(exist_ok=True)
    result = {"project": str(project), "map": str(map_file), "symbols": symbols, "snapshots": []}
    started = time.monotonic()
    index = 0
    while True:
        dump = logs / f"{tag}_{index:03d}.bin"
        command = [str(CLI), "-c", "port=SWD", "sn=35FF72063448303417211657", "freq=1000", "mode=HOTPLUG",
                   "-u", hex(base), str(size), str(dump)]
        capture = subprocess.run(command, capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW, timeout=30)
        output = capture.stdout.decode("utf-8", errors="replace") + capture.stderr.decode("utf-8", errors="replace")
        (logs / f"{tag}_{index:03d}.log").write_text(output, encoding="utf-8")
        if capture.returncode != 0 or not dump.is_file() or dump.stat().st_size != size:
            raise RuntimeError("Probe upload failed; see " + str(dump.with_suffix(".log")))
        raw = dump.read_bytes()
        snapshot = {"elapsed_seconds": round(time.monotonic() - started, 3)}
        for name, keys in (("g_board_link_status", STATUS), ("g_board_link_test", TEST)):
            address, _ = symbols[name]
            words = struct.unpack_from("<" + "I" * len(keys), raw, address - base)
            snapshot[name] = dict(zip(keys, words))
        status = snapshot["g_board_link_status"]
        test = snapshot["g_board_link_test"]
        if status["magic"] != 0x424c4e4b or test["magic"] != 0x54455354:
            raise RuntimeError("Live firmware does not match map: bad link magic")
        expected_node = 1 if project.name == "F1" else 4
        if status["node"] != expected_node or test["node"] != expected_node:
            raise RuntimeError("Connected MCU does not match selected project")
        result["snapshots"].append(snapshot)
        (logs / f"{tag}.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(snapshot), flush=True)
        if time.monotonic() - started >= duration:
            break
        time.sleep(min(interval, max(0, duration - (time.monotonic() - started))))
        index += 1
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--tag", default="board_link_live")
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--interval", type=float, default=10)
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.tag) or args.duration < 0 or args.interval <= 0:
        parser.error("Invalid tag/duration/interval")
    run(args.project.resolve(), args.tag, args.duration, args.interval)
