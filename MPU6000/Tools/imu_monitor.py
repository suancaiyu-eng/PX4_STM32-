#!/usr/bin/env python3
"""Decode MPU6000 telemetry v1 from a serial port or an offline binary log.

Only serial capture needs pyserial. Offline decoding and tests use Python's
standard library. Values retain sensor axes and have no bias compensation.
"""

from __future__ import annotations

import argparse
import binascii
import csv
import dataclasses
import struct
import sys
import time
from pathlib import Path
from typing import BinaryIO, Iterator

SYNC = b"\xa5\x5a"
VERSION = 1
FRAME_SIZE = 64
PAYLOAD = struct.Struct("<IIII7hHBB6I")


def crc16_ccitt_false(data: bytes | bytearray) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection/xorout."""
    return binascii.crc_hqx(data, 0xFFFF)


@dataclasses.dataclass(frozen=True)
class ImuFrame:
    frame_sequence: int
    sample_sequence: int
    timestamp_us: int
    sent_us: int
    accel_x_raw: int
    accel_y_raw: int
    accel_z_raw: int
    gyro_x_raw: int
    gyro_y_raw: int
    gyro_z_raw: int
    temperature_raw: int
    flags: int
    driver_state: int
    driver_error: int
    sample_drops: int
    spi_errors: int
    sample_timeouts: int
    telemetry_drops: int
    uart_errors: int
    uart_timeouts: int

    def csv_row(self) -> dict[str, int | float]:
        row = dataclasses.asdict(self)
        # MPU6000 configured for +/-16 g and +/-2000 deg/s. These are scaled
        # readings in sensor axes, not calibrated aircraft body measurements.
        for axis in "xyz":
            row[f"accel_{axis}_g"] = getattr(self, f"accel_{axis}_raw") / 2048.0
            row[f"gyro_{axis}_dps"] = getattr(self, f"gyro_{axis}_raw") / 16.4
        row["temperature_c"] = self.temperature_raw / 340.0 + 36.53
        row["sample_age_us"] = (self.sent_us - self.timestamp_us) & 0xFFFFFFFF
        row["sample_valid"] = int(bool(self.flags & 0x0001))
        row["sample_stale"] = int(bool(self.flags & 0x0004))
        return row


class FrameDecoder:
    """Incremental decoder; reject corrupt headers/CRC and find the next sync.

    At most 63 unparsed bytes remain after a feed call. A missing frame's gap
    may come from firmware scheduling, UART loss, or host capture; these cannot
    be distinguished by the frame sequence alone. Sample-sequence jumps around
    10 are expected for 1 kHz sampling / 100 Hz telemetry.
    """

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.header_errors = 0
        self.discarded_bytes = 0
        self.missing_frame_slots = 0
        self.sequence_resets = 0
        self._last_sequence: int | None = None

    def feed(self, data: bytes) -> list[ImuFrame]:
        self.buffer.extend(data)
        result = []
        while self.buffer:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                keep = 1 if self.buffer[-1] == SYNC[0] else 0
                discard = len(self.buffer) - keep
                self.discarded_bytes += discard
                del self.buffer[:discard]
                break
            if sync_index:
                self.discarded_bytes += sync_index
                del self.buffer[:sync_index]
            if len(self.buffer) < 4:
                break
            if self.buffer[2] != VERSION or self.buffer[3] != FRAME_SIZE:
                self.header_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            expected = struct.unpack_from("<H", self.buffer, 62)[0]
            if crc16_ccitt_false(self.buffer[2:62]) != expected:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            frame = ImuFrame(*PAYLOAD.unpack_from(self.buffer, 4))
            del self.buffer[:FRAME_SIZE]
            result.append(frame)
            self.frames_ok += 1
            if self._last_sequence is not None:
                delta = (frame.frame_sequence - self._last_sequence) & 0xFFFFFFFF
                if 1 < delta < 0x80000000:
                    self.missing_frame_slots += delta - 1
                elif delta == 0 or delta >= 0x80000000:
                    self.sequence_resets += 1
            self._last_sequence = frame.frame_sequence
        return result

    def summary(self) -> str:
        return (
            f"frames={self.frames_ok} crc_errors={self.crc_errors} "
            f"header_errors={self.header_errors} discarded_bytes={self.discarded_bytes} "
            f"missing_frame_slots={self.missing_frame_slots} "
            f"sequence_resets={self.sequence_resets} trailing_bytes={len(self.buffer)}"
        )


def read_chunks(source: BinaryIO, serial_mode: bool,
                duration: float | None) -> Iterator[bytes]:
    deadline = None if duration is None else time.monotonic() + duration
    while deadline is None or time.monotonic() < deadline:
        chunk = source.read(4096)
        if chunk:
            yield chunk
        elif not serial_mode:
            break


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--port", help="serial port, e.g. COM5")
    source_group.add_argument("--input", type=Path, help="offline binary capture")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--output", type=Path, help="CSV file; default is stdout")
    parser.add_argument("--raw-log", type=Path, help="also save received raw bytes")
    parser.add_argument("--duration", type=float, help="stop after this many seconds")
    args = parser.parse_args(argv)
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    input_path = args.input.resolve() if args.input else None
    output_paths = [p.resolve() for p in (args.output, args.raw_log) if p]
    if (input_path is not None and input_path in output_paths) or len(set(output_paths)) != len(output_paths):
        parser.error("input, CSV output and raw log must be different files")

    if args.port:
        try:
            import serial
        except ImportError:
            parser.error("serial capture needs pyserial: python -m pip install pyserial")
        source = serial.Serial(args.port, args.baud, timeout=0.1,
                               bytesize=8, parity="N", stopbits=1,
                               xonxoff=False, rtscts=False, dsrdtr=False)
    else:
        source = args.input.open("rb")

    output = None
    raw_log = None
    decoder = FrameDecoder()
    writer = None
    try:
        output = args.output.open("w", newline="", encoding="utf-8") if args.output else sys.stdout
        raw_log = args.raw_log.open("wb") if args.raw_log else None
        for chunk in read_chunks(source, bool(args.port), args.duration):
            if raw_log:
                raw_log.write(chunk)
            for frame in decoder.feed(chunk):
                row = frame.csv_row()
                if writer is None:
                    writer = csv.DictWriter(output, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
            output.flush()
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        if raw_log:
            raw_log.close()
        if output is not None and output is not sys.stdout:
            output.close()
        print(decoder.summary(), file=sys.stderr)
    return 0 if decoder.frames_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
