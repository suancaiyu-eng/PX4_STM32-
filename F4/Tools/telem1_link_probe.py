"""Read running F4 TELEM1/link counters and formatted lines through ST-Link.

This checks target-side DMA completion, not computer serial reception.
"""
from pathlib import Path
import json
import re
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CLI = r"D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
map_text = (ROOT / "MDK-ARM/MPU6000/MPU6000.map").read_text(errors="replace")
names = ("g_telemetry_status", "g_board_link_telem_status", "g_board_link_status", "g_board_link_test", "text_queue")
symbols = {}
for name in names:
    match = re.search(r"(?m)^\s+" + name + r"\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)", map_text)
    if not match:
        raise RuntimeError("Missing map symbol " + name)
    symbols[name] = (int(match[1], 16), int(match[2]))
base = min(address for address, _ in symbols.values())
size = max(address + length for address, length in symbols.values()) - base
if not (0x20000000 <= base < base + size <= 0x20030000):
    raise RuntimeError("Unexpected SRAM addresses")
result = {"verification": "Target-side DMA and board link; not a PC serial capture", "symbols": symbols, "snapshots": []}
for index in range(3):
    if index:
        time.sleep(10)
    dump = ROOT / f"Logs/telem1_link_verified_{index}.bin"
    completed = subprocess.run([CLI, "-c", "port=SWD", "sn=35FF72063448303417211657", "freq=1000", "mode=HOTPLUG",
                                "-u", hex(base), str(size), str(dump)], capture_output=True,
                               creationflags=subprocess.CREATE_NO_WINDOW, timeout=30)
    dump.with_suffix(".log").write_bytes(completed.stdout + completed.stderr)
    if completed.returncode != 0 or not dump.is_file() or dump.stat().st_size != size:
        raise RuntimeError("SRAM read failed")
    raw = dump.read_bytes()
    def words(name):
        address, length = symbols[name]
        return struct.unpack_from("<" + "I" * (length // 4), raw, address - base)
    telemetry = words("g_telemetry_status")
    display = words("g_board_link_telem_status")
    link = words("g_board_link_status")
    tests = words("g_board_link_test")
    sample = {"index": index, "uptime_ms": link[2], "text_received": display[0], "text_queued": telemetry[13],
              "text_started": telemetry[14], "text_dropped": telemetry[15], "dma_started": telemetry[0],
              "dma_completed": telemetry[1], "imu_frames_dropped": telemetry[2], "uart_errors": telemetry[3],
              "tx_timeouts": telemetry[4], "start_errors": telemetry[5], "faulted": (telemetry[10] >> 8) & 255,
              "link_crc_errors": link[7], "link_uart_errors": link[9], "matched_replies": tests[3],
              "reply_timeouts": tests[5], "peer_online": tests[10]}
    address, length = symbols["text_queue"]
    queue = raw[address - base:address - base + length]
    lines = []
    for start in range(0, length, 320):
        line = queue[start:start + 320].split(b"\n", 1)[0]
        if line.startswith(b"#LINK_RX "):
            lines.append(line.decode("ascii"))
    sample["formatted_lines"] = lines
    result["snapshots"].append(sample)
    (ROOT / "Logs/telem1_link_verified.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(sample), flush=True)
first, last = result["snapshots"][0], result["snapshots"][-1]
assert last["text_started"] > first["text_started"] + 100
assert last["matched_replies"] > first["matched_replies"] + 50 and last["peer_online"] == 1
for key in ("text_dropped", "uart_errors", "tx_timeouts", "start_errors", "link_crc_errors", "link_uart_errors", "reply_timeouts"):
    assert last[key] == first[key], key
assert last["faulted"] == 0 and last["formatted_lines"]
print("PASS: TELEM1 link lines reach the DMA transmitter, no new errors/drops, real F1/F4 communication remains online", flush=True)
