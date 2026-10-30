"""Measure real R9DS inputs from TELEM1 RC events or live F1 SRAM.

No calibration values are generated from no-data/failsafe/timeout samples.
--phase neutral: controls held at mechanical center (switches at default).
--phase range: user sweeps every stick/knob/switch through its full travel.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import re
import statistics
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CLI = r"D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
FIELDS = ("last_frame_ms age_ms frame_count uart_errors invalid_frames queue_drops "
          "buffer_overflows frame_lost_reports failsafe_reports").split()
STATES = ("NO_DATA", "OK", "FRAME_LOST", "FAILSAFE", "TIMEOUT")


def packet(payload, sequence=None):
    if len(payload) != 60:
        raise ValueError("RC payload must have 60 bytes")
    words = struct.unpack("<4B9I10H", payload)
    if words[0] != 1 or words[1] > 4 or words[3] != 10 or words[2] & 0xc0:
        raise ValueError("Invalid RC metadata")
    if any(value > 2047 for value in words[13:]):
        raise ValueError("RC channel outside eleven-bit range")
    result = dict(zip(FIELDS, words[4:13]))
    result.update(status=STATES[words[1]], flags=words[2], channels=list(words[13:]), sequence=sequence)
    result["valid"] = words[1] == 1 and words[2] & 3 == 3 and words[2] & 12 == 0 and result["age_ms"] < 100
    return result


def load_serial(path):
    records = []
    expression = re.compile(r"^#LINK_RX src=F1 type=EVENT cmd=0x0200 seq=(\d+) len=60 .* hex=([0-9A-F]{120})$", re.M)
    for match in expression.finditer(path.read_text(errors="replace").replace("\r", "")):
        records.append(packet(bytes.fromhex(match[2]), int(match[1])))
    return records


def capture_swd(path, duration):
    text = (ROOT / "MDK-ARM/F1/F1.map").read_text(errors="replace")
    match = re.search(r"(?m)^\s+g_r9ds\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)", text)
    if not match or int(match[2]) != 100:
        raise RuntimeError("Unexpected F1 snapshot layout; rebuild/map mismatch")
    address = int(match[1], 16)
    dump = path.with_suffix(".bin")
    log = path.with_suffix(".probe.log")
    started = time.monotonic()
    records = []
    with path.open("w", encoding="utf-8") as output:
        while time.monotonic() - started < duration:
            run = subprocess.run([CLI, "-c", "port=SWD", "sn=35FF72063448303417211657", "freq=1000", "mode=HOTPLUG",
                                  "-u", hex(address), "100", str(dump)], capture_output=True,
                                 creationflags=subprocess.CREATE_NO_WINDOW, timeout=15)
            log.write_bytes(run.stdout + run.stderr)
            if (run.returncode != 0 or not re.search(rb"Device ID\s*:\s*0x410\b", run.stdout)
                    or not dump.is_file() or dump.stat().st_size != 100):
                raise RuntimeError("F1 SRAM read failed; check the probe log")
            raw = dump.read_bytes()
            channel = struct.unpack_from("<16H", raw, 0)[:10]
            flags = sum((1 << i) if raw[offset] else 0 for i, offset in enumerate((37, 38, 35, 36, 33, 34)))
            last_frame, age = struct.unpack_from("<II", raw, 40)
            counters = struct.unpack_from("<13I", raw, 48)
            values = (1, raw[39], flags, raw[32], last_frame, age, counters[0], counters[5], counters[1],
                      counters[4], counters[3], counters[11], counters[12], *channel)
            record = packet(struct.pack("<4B9I10H", *values))
            record["elapsed_seconds"] = round(time.monotonic() - started, 3)
            records.append(record)
            output.write(json.dumps(record) + "\n")
            output.flush()
            if len(records) == 1 or len(records) % 10 == 0:
                print(json.dumps({"seconds": record["elapsed_seconds"], "status": record["status"],
                                  "frames": record["frame_count"], "channels": record["channels"]}), flush=True)
            time.sleep(0.05)
    return records


def analyze(records, phase, neutral_path, output, write_header):
    if not records:
        raise RuntimeError("No RC records captured; check port/firmware and cmd=0x0200")
    good = [row for row in records if row["valid"]]
    errors = {key: (records[-1][key] - records[0][key]) & 0xffffffff for key in FIELDS[3:]}
    sequence = [row["sequence"] for row in records if row["sequence"] is not None]
    gaps = sum(max(0, ((b-a) & 65535)-1) for a,b in zip(sequence, sequence[1:]))
    result = {"phase": phase, "time": datetime.now(timezone(timedelta(hours=8))).isoformat(),
              "samples": len(records), "valid_samples": len(good), "states": dict(Counter(row["status"] for row in records)),
              "error_increments": errors, "telemetry_sequence_gaps": gaps, "latest": records[-1],
              "frame_increment": (records[-1]["frame_count"] - records[0]["frame_count"]) & 0xffffffff}
    result["healthy"] = len(good) >= 10 and len(good) / len(records) >= 0.98 and not any(errors.values()) and result["frame_increment"] > 0
    if phase != "health" and not result["healthy"]:
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        raise RuntimeError("Capture is not healthy; calibration refused (see report)")
    if phase == "neutral":
        result["neutral"] = [int(statistics.median(row["channels"][i] for row in good)) for i in range(10)]
        result["jitter_span"] = [max(row["channels"][i] for row in good)-min(row["channels"][i] for row in good) for i in range(10)]
        if max(result["jitter_span"]) > 40:
            raise RuntimeError("Controls moved or input was noisy during neutral capture; hold still and repeat")
    if phase == "range":
        neutral = json.loads(neutral_path.read_text(encoding="utf-8"))
        if neutral.get("phase") != "neutral" or not neutral.get("healthy"):
            raise RuntimeError("A healthy neutral capture is required first")
        channels = []
        mask = 0
        for i in range(10):
            values = [row["channels"][i] for row in good]
            low, high = min(values), max(values)
            span = high - low
            reference = neutral["neutral"][i]
            interior = low + span * 0.15 <= reference <= high - span * 0.15
            center = reference if interior else (low + high) // 2
            low_hits = sum(value <= low + 4 for value in values)
            high_hits = sum(value >= high - 4 for value in values)
            valid = span >= 200 and low_hits >= 3 and high_hits >= 3 and low < center < high
            if valid: mask |= 1 << i
            channels.append({"channel": i+1, "min": low, "center": center, "max": high, "span": span,
                             "neutral": reference, "center_method": "measured-neutral" if interior else "range-midpoint",
                             "min_hits": low_hits, "max_hits": high_hits, "calibrated": valid})
        result["channels"] = channels
        result["valid_mask"] = mask
        if not mask:
            output.write_text(json.dumps(result, indent=2), encoding="utf-8")
            raise RuntimeError("No channel traversed a usable range; move controls through all endpoints and repeat")
        if write_header:
            lines = ["#ifndef R9DS_CALIBRATION_DATA_H", "#define R9DS_CALIBRATION_DATA_H",
                     "/* Measured R9DS calibration. Unmeasured channels remain invalid. */",
                     f"#define R9DS_CALIBRATION_VALID_MASK 0x{mask:04X}U"]
            for name, key in (("min", "min"), ("center", "center"), ("max", "max")):
                lines.append(f"static const uint16_t r9ds_cal_{name}[10] = {{" + ", ".join(str(row[key]) for row in channels) + "};")
            lines.extend(["#endif", ""])
            (ROOT / "Core/Inc/r9ds_calibration_data.h").write_text("\n".join(lines), encoding="utf-8")
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase", choices=("health", "neutral", "range"), default="health")
    parser.add_argument("--swd", action="store_true")
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--neutral", type=Path, default=ROOT / "Logs/rc_neutral.json")
    parser.add_argument("--write-header", action="store_true")
    args = parser.parse_args()
    if args.swd:
        records = capture_swd(args.input, args.duration)
    elif args.input.suffix == ".jsonl":
        records = [json.loads(line) for line in args.input.read_text().splitlines() if line]
    else:
        records = load_serial(args.input)
    analyze(records, args.phase, args.neutral, args.output, args.write_header)
