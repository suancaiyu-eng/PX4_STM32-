"""Host-side protocol checks; no serial hardware or third-party modules needed."""

import csv
import importlib.util
import io
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "Tools" / "imu_monitor.py"
SPEC = importlib.util.spec_from_file_location("imu_monitor", MODULE_PATH)
monitor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = monitor
SPEC.loader.exec_module(monitor)


def frame_bytes(sequence=1, sample_sequence=10, timestamp_us=10000,
                sent_us=10100, flags=0x13):
    payload = monitor.PAYLOAD.pack(
        sequence, sample_sequence, timestamp_us, sent_us,
        -32768, 0, 2048, -164, 164, 32767, -340,
        flags, 3, 0, 2, 3, 4, 5, 6, 7,
    )
    prefix = monitor.SYNC + bytes((1, 64)) + payload
    return prefix + struct.pack("<H", monitor.crc16_ccitt_false(prefix[2:]))


class ProtocolTests(unittest.TestCase):
    def test_known_crc_vector(self):
        self.assertEqual(monitor.crc16_ccitt_false(b"123456789"), 0x29B1)
        self.assertEqual(monitor.crc16_ccitt_false(b""), 0xFFFF)

    def test_decode_and_scale(self):
        decoder = monitor.FrameDecoder()
        wire = frame_bytes()
        self.assertEqual(len(wire), 64)
        frame, = decoder.feed(wire)
        self.assertEqual(frame.accel_x_raw, -32768)
        self.assertEqual(frame.gyro_z_raw, 32767)
        self.assertEqual(frame.sample_drops, 2)
        self.assertEqual(frame.uart_timeouts, 7)
        row = frame.csv_row()
        self.assertEqual(row["accel_z_g"], 1.0)
        self.assertAlmostEqual(row["gyro_x_dps"], -10.0)
        self.assertAlmostEqual(row["temperature_c"], 35.53)
        self.assertEqual(row["sample_age_us"], 100)
        self.assertEqual(row["sample_valid"], 1)

    def test_every_split_position(self):
        wire = frame_bytes()
        for split in range(len(wire) + 1):
            with self.subTest(split=split):
                decoder = monitor.FrameDecoder()
                frames = decoder.feed(wire[:split]) + decoder.feed(wire[split:])
                self.assertEqual(len(frames), 1)
                self.assertEqual(frames[0].sample_sequence, 10)
                self.assertEqual(decoder.buffer, bytearray())

    def test_junk_and_sync_split(self):
        decoder = monitor.FrameDecoder()
        self.assertEqual(decoder.feed(b"garbage\xa5"), [])
        self.assertEqual(len(decoder.buffer), 1)
        frames = decoder.feed(frame_bytes()[1:] + b"tail")
        self.assertEqual(len(frames), 1)
        self.assertEqual(decoder.discarded_bytes, 11)

    def test_corrupt_crc_and_header_resynchronize(self):
        corrupt = bytearray(frame_bytes(2))
        corrupt[20] ^= 1
        bad_header = monitor.SYNC + b"\x02\x40" + b"junk"
        bad_length = monitor.SYNC + b"\x01\xff" + b"junk"
        decoder = monitor.FrameDecoder()
        frames = decoder.feed(frame_bytes(1) + corrupt + bad_header + bad_length + frame_bytes(4))
        self.assertEqual([f.frame_sequence for f in frames], [1, 4])
        self.assertEqual(decoder.crc_errors, 1)
        self.assertEqual(decoder.header_errors, 2)
        self.assertEqual(decoder.missing_frame_slots, 2)

    def test_truncated_frame_does_not_swallow_next_frame(self):
        decoder = monitor.FrameDecoder()
        frames = decoder.feed(frame_bytes(1)[:25] + frame_bytes(2))
        self.assertEqual([f.frame_sequence for f in frames], [2])
        self.assertGreaterEqual(decoder.crc_errors, 1)

    def test_wraparound_and_random_chunks(self):
        stream = b"junk" + b"".join(frame_bytes(i) for i in [0xFFFFFFFE, 0xFFFFFFFF, 0, 1])
        decoder = monitor.FrameDecoder()
        rng = random.Random(6000)
        frames = []
        while stream:
            size = rng.randint(1, 30)
            frames.extend(decoder.feed(stream[:size]))
            stream = stream[size:]
        self.assertEqual(len(frames), 4)
        self.assertEqual(decoder.missing_frame_slots, 0)
        self.assertEqual(decoder.sequence_resets, 0)

    def test_sample_age_wrap_and_invalid_flags(self):
        decoder = monitor.FrameDecoder()
        frame, = decoder.feed(frame_bytes(timestamp_us=0xFFFFFFF0, sent_us=16, flags=0x0C))
        self.assertEqual(frame.csv_row()["sample_age_us"], 32)
        self.assertEqual(frame.csv_row()["sample_valid"], 0)
        self.assertEqual(frame.csv_row()["sample_stale"], 1)

    def test_offline_cli_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            raw_path = Path(directory) / "capture.bin"
            csv_path = Path(directory) / "decoded.csv"
            raw_path.write_bytes(frame_bytes())
            with redirect_stderr(io.StringIO()):
                status = monitor.main(["--input", str(raw_path), "--output", str(csv_path)])
            self.assertEqual(status, 0)
            with csv_path.open(newline="", encoding="utf-8") as source:
                row, = list(csv.DictReader(source))
            self.assertEqual(row["sample_sequence"], "10")
            self.assertEqual(row["sample_valid"], "1")

    def test_reject_input_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.bin"
            original = frame_bytes()
            path.write_bytes(original)
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                monitor.main(["--input", str(path), "--output", str(path)])
            self.assertEqual(path.read_bytes(), original)


class NativeTelemetryTests(unittest.TestCase):
    """Run the actual telemetry C source against a deliberately small HAL fake."""

    def run_native(self, binary):
        project = MODULE_PATH.parents[1]
        stub = r'''
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { uint32_t CR; } DMA_Stream_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmatx; uint32_t ErrorCode;
                 struct { uint32_t BaudRate; } Init; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define DMA_SxCR_EN 1U
#define __NVIC_PRIO_BITS 4U
extern uint32_t test_basepri;
static inline uint32_t __get_BASEPRI(void) { return test_basepri; }
static inline void __set_BASEPRI(uint32_t p) { test_basepri = p; }
static inline void __set_BASEPRI_MAX(uint32_t p) {
  if (!test_basepri || (p && p < test_basepri)) test_basepri = p;
}
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *);
#endif
'''
        harness = r'''
#include "imu_telemetry.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
uint32_t test_basepri;
static DMA_Stream_TypeDef stream;
static DMA_HandleTypeDef dma = { &stream };
static UART_HandleTypeDef uart = { &dma, 0, {460800U} };
static UART_HandleTypeDef other_uart = { &dma, 0, {460800U} };
static const uint8_t *live_buffer;
static uint16_t live_length;
static unsigned starts, aborts, immediate_abort;
static HAL_StatusTypeDef start_result;

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,
                                      const uint8_t *p, uint16_t n) {
  assert(u == &uart);
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_BINARY
  assert(n == 64);
#else
  assert(n > 8 && n < IMU_TELEMETRY_TX_BUFFER_SIZE && p[n - 1] == '\n');
  if (p[0] == '#') assert(memcmp(p, "#IMU ", 5) == 0);
  else assert(memcmp(p, "mpu6000:", 8) == 0);
  assert(memchr(p, 0, n) == 0); /* no transmitted NUL terminator */
#endif
  /* Priority 5 remains unmasked: BASEPRI must be priority 8 (or inherited). */
  assert(test_basepri && test_basepri <= 0x80U);
  ++starts;
  if (start_result == HAL_OK) { stream.CR = 1; live_buffer = p; live_length = n; }
  return start_result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *u) {
  assert(u == &uart && test_basepri && test_basepri <= 0x80U);
  ++aborts;
  if (immediate_abort) {
    stream.CR = 0;
    ImuTelemetry_OnAbortComplete(u);
  }
  return HAL_OK;
}
static void reset(uint32_t now) {
  starts = aborts = immediate_abort = 0;
  stream.CR = test_basepri = 0;
  start_result = HAL_OK;
  live_buffer = 0;
  ImuTelemetry_Init(&uart, now);
}
static ImuTelemetryStats stats(void) {
  ImuTelemetryStats s;
  ImuTelemetry_GetStats(&s);
  assert(test_basepri == 0);
  return s;
}
int main(void) {
  ImuTelemetryRecord r = {0};
  ImuTelemetryStats s;
  uint8_t snapshot[IMU_TELEMETRY_TX_BUFFER_SIZE];
  unsigned i;
  r.timestamp_us = 10000;
  r.sequence = 10;
  r.accel_raw[0] = -32768; r.accel_raw[2] = 2048;
  r.gyro_raw[0] = -164; r.gyro_raw[1] = 164; r.gyro_raw[2] = 32767;
  r.temperature_raw = -340; r.flags = 0x13; r.driver_state = 3;
  r.sample_drops = 2; r.spi_errors = 3; r.timeouts = 4;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_BINARY
  /* A live estimator must not change a single binary-v1 byte or its CRC. */
  r.attitude_present = 1; r.attitude_flags = 15; r.attitude_timestamp_us = 10000;
  r.attitude_mdeg[0] = -12345; r.attitude_state = 2;
#endif

  reset(100);
  ImuTelemetry_Service(&r, 10099); assert(starts == 0);
  ImuTelemetry_Service(&r, 10100); assert(starts == 1);
  /* Python decodes this actual C-produced packet and checks its CRC. */
  for (i = 0; i < live_length; ++i) printf("%02x", live_buffer[i]);
  printf("\n");
  memcpy(snapshot, live_buffer, 64);
  r.sequence = 99;
  ImuTelemetry_Service(&r, 20100);
  assert(starts == 1 && memcmp(snapshot, live_buffer, 64) == 0);
  s = stats(); assert(s.busy && s.frames_dropped == 1);
  ImuTelemetry_OnTxComplete(&other_uart); assert(stats().busy);
  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  assert(stats().frames_completed == 1);
  ImuTelemetry_Service(&r, 30100); assert(starts == 2);

  /* In-progress abort cannot free the DMA buffer, even on a late TX callback. */
  reset(0); ImuTelemetry_Service(&r, 10000);
  memcpy(snapshot, live_buffer, 64);
  ImuTelemetry_Service(&r, 30000); assert(aborts == 1 && starts == 1);
  s = stats(); assert(s.tx_timeouts == 1 && s.busy);
  ImuTelemetry_OnTxComplete(&uart); assert(stats().busy);
  r.sequence = 100;
  ImuTelemetry_Service(&r, 40000);
  assert(memcmp(snapshot, live_buffer, 64) == 0);
  stream.CR = 0; ImuTelemetry_OnAbortComplete(&uart);
  assert(!stats().busy);
  ImuTelemetry_Service(&r, 50000); assert(starts == 2);

  /* Unconfirmed abort latches a fault and never reuses DMA-owned RAM. */
  reset(0); ImuTelemetry_Service(&r, 10000);
  memcpy(snapshot, live_buffer, 64);
  ImuTelemetry_Service(&r, 30000);
  ImuTelemetry_Service(&r, 50000);
  s = stats(); assert(s.faulted && s.abort_timeouts == 1);
  stream.CR = 0; ImuTelemetry_OnAbortComplete(&uart);
  ImuTelemetry_OnTxComplete(&uart);
  ImuTelemetry_Service(&r, 60000);
  assert(starts == 1 && memcmp(snapshot, live_buffer, 64) == 0);

  /* HAL may invoke abort completion synchronously: state must be ready first. */
  reset(0); immediate_abort = 1;
  ImuTelemetry_Service(&r, 10000);
  uart.ErrorCode = 0x20; ImuTelemetry_OnError(&uart);
  ImuTelemetry_Service(&r, 10001);
  s = stats(); assert(aborts == 1 && !s.busy && s.uart_errors == 1);
  assert(s.last_hal_error == 0x20);
  ImuTelemetry_Service(&r, 20000); assert(starts == 2);

  reset(0); start_result = HAL_ERROR;
  ImuTelemetry_Service(&r, 10000);
  s = stats(); assert(s.start_errors == 1 && s.frames_dropped == 1 && !s.busy);

  reset(0); start_result = HAL_BUSY; stream.CR = 1;
  ImuTelemetry_Service(&r, 10000); assert(stats().faulted);

  reset(0xFFFFF000U);
  ImuTelemetry_Service(&r, 0xFFFFF000U + 10000U); assert(starts == 1);

  reset(0); ImuTelemetry_Service(0, 10000);
  assert(starts == 0 && stats().frames_dropped == 1);
  ImuTelemetry_Init(0, 0); ImuTelemetry_Service(&r, 10000);
  assert(stats().faulted);

#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
  /* Invalid records emit a diagnostic, never fake numeric measurements. */
  reset(0); r.flags = 0;
  r.driver_state = 7; r.driver_error = 1; r.who_am_i = 0xFF;
  r.bad_register = 0x75; r.spi_errors = 2; r.timeouts = 3;
  ImuTelemetry_Service(&r, 10000); assert(starts == 1);
  assert(strcmp((const char *)live_buffer,
    "#IMU state=7 error=1 id=0xFF reg=0x75 wr=00 rd=00 drdy=0 samples=0 spi=2 timeout=3\n") == 0);
  assert(stats().status_frames == 1);
  memcpy(snapshot, live_buffer, 64);
  r.flags = IMU_TELEMETRY_FLAG_SAMPLE_VALID | IMU_TELEMETRY_FLAG_STALE;
  ImuTelemetry_Service(&r, 20000);
  assert(starts == 1 && memcmp(snapshot, live_buffer, 64) == 0);
  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  r.flags = IMU_TELEMETRY_FLAG_SAMPLE_VALID | IMU_TELEMETRY_FLAG_DRIVER_FAULT;
  ImuTelemetry_Service(&r, 30000); assert(starts == 1);
  assert(stats().frames_dropped == 2);
  ImuTelemetry_Service(&r, 1009999); assert(starts == 1);
  ImuTelemetry_Service(&r, 1010000); assert(starts == 2);
  assert(stats().status_frames == 2);

  /* A diagnostic with all uint32 counters at their maximum still fits. */
  reset(0);
  r.driver_state = r.driver_error = 255; r.bad_register = r.who_am_i = 255;
  r.readback_expected = r.readback_actual = 255;
  r.drdy_count = r.samples = r.spi_errors = r.timeouts = 0xFFFFFFFFU;
  ImuTelemetry_Service(&r, 10000);
  assert(strcmp((const char *)live_buffer,
    "#IMU state=255 error=255 id=0xFF reg=0xFF wr=FF rd=FF drdy=4294967295 samples=4294967295 spi=4294967295 timeout=4294967295\n") == 0);

  /* Diagnostic scheduling also handles the microsecond clock wrapping. */
  reset(0xFFFFF000U);
  ImuTelemetry_Service(&r, 0xFFFFF000U + 10000U); assert(starts == 1);
  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  ImuTelemetry_Service(&r, 0xFFFFF000U + 20000U); assert(starts == 1);
  ImuTelemetry_Service(&r, 0xFFFFF000U + 1010000U); assert(starts == 2);

  /* Every int16 minimum is the longest possible line: no overflow/truncation. */
  reset(0);
  r.flags = IMU_TELEMETRY_FLAG_SAMPLE_VALID;
  for (i = 0; i < 3; ++i) r.accel_raw[i] = r.gyro_raw[i] = -32768;
  r.temperature_raw = -32768;
  ImuTelemetry_Service(&r, 40000);
  assert(starts == 1 && live_length == 57);
  assert(memcmp(live_buffer,
    "mpu6000:-32768,-32768,-32768,-32768,-32768,-32768,-32768\n", 57) == 0);

  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  memset(&r, 0, sizeof(r)); r.flags = IMU_TELEMETRY_FLAG_SAMPLE_VALID;
  ImuTelemetry_Service(&r, 50000);
  assert(live_length == 22 && memcmp(live_buffer, "mpu6000:0,0,0,0,0,0,0\n", 22) == 0);

  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  for (i = 0; i < 3; ++i) r.accel_raw[i] = r.gyro_raw[i] = 32767;
  r.temperature_raw = 32767;
  ImuTelemetry_Service(&r, 60000);
  assert(live_length == 50 && memcmp(live_buffer,
    "mpu6000:32767,32767,32767,32767,32767,32767,32767\n", 50) == 0);

  /* Valid attitude shares one DMA-owned frame with the unchanged raw line.
   * Check signed zero fractions, exact extrema, ownership and freshness. */
  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  r.attitude_present = 1; r.attitude_flags = 1;
  r.attitude_timestamp_us = 70000;
  r.attitude_mdeg[0] = -180000; r.attitude_mdeg[1] = -1; r.attitude_mdeg[2] = 180000;
  ImuTelemetry_Service(&r, 70000);
  assert(strcmp((const char *)live_buffer,
    "mpu6000:32767,32767,32767,32767,32767,32767,32767,-180.000,-0.001,180.000,1\n") == 0);
  assert(stats().attitude_frames == 1);
  memcpy(snapshot, live_buffer, sizeof(snapshot));
  r.attitude_mdeg[0] = 12345;
  ImuTelemetry_Service(&r, 80000);
  assert(memcmp(snapshot, live_buffer, sizeof(snapshot)) == 0);
  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  r.attitude_state = 1; r.calibration_samples = 1999; r.calibration_restarts = 2;
  ImuTelemetry_Service(&r, 90000); /* old attitude must not be transmitted */
  assert(strstr((const char *)live_buffer, "attitude:") == 0);
  assert(strstr((const char *)live_buffer, ",0.000,0.000,0.000,0\n#ATT") != 0);
  assert(strstr((const char *)live_buffer, "#ATT state=1 error=0 cal=1999 reset=2\n") != 0);
  assert(stats().attitude_status_frames == 1);
  stream.CR = 0; ImuTelemetry_OnTxComplete(&uart);
  r.attitude_flags = 0;
  ImuTelemetry_Service(&r, 100000);
  assert(live_length == 70); /* startup/fault diagnostics throttled, raw continues */

  /* Publication and diagnostic timers both survive the microsecond wrap. */
  reset(0xfffff000U); r.attitude_flags = 1;
  r.attitude_timestamp_us = 0xfffff000U;
  r.attitude_mdeg[0] = 0; r.attitude_mdeg[1] = 90000; r.attitude_mdeg[2] = -90000;
  ImuTelemetry_Service(&r, 0xfffff000U + 10000U);
  assert(strstr((const char *)live_buffer, ",0.000,90.000,-90.000,1\n") != 0);

  /* Maximum-width diagnostics still preserve every required numeric field. */
  reset(0); r.attitude_flags = 0;
  r.attitude_state = r.attitude_error = r.calibration_samples = r.calibration_restarts = 0xffffffffU;
  for (i = 0; i < 3; ++i) r.accel_raw[i] = r.gyro_raw[i] = -32768;
  r.temperature_raw = -32768;
  ImuTelemetry_Service(&r, 10000);
  assert(live_length > 77 && live_length < IMU_TELEMETRY_TX_BUFFER_SIZE && stats().attitude_status_frames == 1);
  assert(memcmp(live_buffer,
    "mpu6000:-32768,-32768,-32768,-32768,-32768,-32768,-32768,0.000,0.000,0.000,0\n", 77) == 0);

  reset(0); memset(&r,0,sizeof(r)); r.flags=IMU_TELEMETRY_FLAG_SAMPLE_VALID;
  r.attitude_present=r.mag_present=r.heading_valid=r.mag_calibrated=r.mag_used=r.mag_healthy=1;
  r.attitude_flags=1; r.attitude_timestamp_us=10000;
  r.attitude_mdeg[0]=1234; r.attitude_mdeg[1]=-2345; r.attitude_mdeg[2]=6789;
  r.mag_raw_mut[0]=-35600; r.mag_raw_mut[1]=12008; r.mag_raw_mut[2]=49000;
  ImuTelemetry_Service(&r,10000);
  assert(strcmp((const char *)live_buffer,
    "mpu6000:0,0,0,0,0,0,0,1.234,-2.345,6.789,1,-35.600,12.008,49.000,1,1,1,1\n")==0);
  stream.CR=0; ImuTelemetry_OnTxComplete(&uart);
  r.attitude_timestamp_us=0;
  ImuTelemetry_Service(&r,30000);
  assert(strstr((const char *)live_buffer,
    ",0.000,0.000,0.000,0,-35.600,12.008,49.000,0,1,0,1\n")!=0);
  stream.CR=0; ImuTelemetry_OnTxComplete(&uart);
  r.attitude_timestamp_us=40000; r.mag_healthy=0;
  ImuTelemetry_Service(&r,40000);
  assert(strstr((const char *)live_buffer,",0.000,0.000,0.000,0,1,0,0\n")!=0);
#endif
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "stm32f4xx_hal.h").write_text(stub, encoding="utf-8")
            (path / "test.c").write_text(harness, encoding="utf-8")
            executable = path / "test.exe"
            # Exercise retained IMU formats explicitly; RC-only console silence
            # has separate coverage in test_telem1_link.py.
            format_option = [f"-DIMU_TELEMETRY_OUTPUT_FORMAT={0 if binary else 1}"]
            build = subprocess.run(
                [shutil.which("gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-DIMU_TELEMETRY_RC_ONLY=0",
                 *format_option,
                 "-I", str(path), "-I", str(project / "Core" / "Inc"),
                 str(project / "Core" / "Src" / "imu_telemetry.c"),
                 str(path / "test.c"), "-o", str(executable)],
                capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True,
                                 text=True, timeout=10)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            wire = bytes.fromhex(run.stdout.strip())
        return wire

    @unittest.skipUnless(shutil.which("gcc"), "native gcc is needed for the C state-machine test")
    def test_dma_ownership_timeout_and_wire_encoding(self):
        wire = self.run_native(binary=True)
        decoder = monitor.FrameDecoder()
        frame, = decoder.feed(wire)
        self.assertEqual(decoder.crc_errors, 0)
        self.assertEqual(frame.frame_sequence, 1)
        self.assertEqual(frame.sample_sequence, 10)
        self.assertEqual(frame.timestamp_us, 10000)
        self.assertEqual(frame.sent_us, 10100)
        self.assertEqual(frame.accel_x_raw, -32768)
        self.assertEqual(frame.gyro_z_raw, 32767)
        self.assertEqual(frame.temperature_raw, -340)
        self.assertEqual(frame.flags, 0x13)
        self.assertEqual(frame.sample_drops, 2)
        self.assertEqual(frame.spi_errors, 3)
        self.assertEqual(frame.sample_timeouts, 4)
        self.assertEqual(frame.telemetry_drops, 0)

    @unittest.skipUnless(shutil.which("gcc"), "native gcc is needed for the C state-machine test")
    def test_firewater_raw_channels_dma_and_fault_gating(self):
        wire = self.run_native(binary=False)
        self.assertEqual(wire, b"mpu6000:-32768,0,2048,-164,164,32767,-340\n")
        name, channels = wire.decode("ascii").rstrip("\n").split(":")
        self.assertEqual(name, "mpu6000")
        self.assertEqual([int(value) for value in channels.split(",")],
                         [-32768, 0, 2048, -164, 164, 32767, -340])


if __name__ == "__main__":
    unittest.main()
