"""Compile the real MPU6000 driver against a deterministic sensor/port fake.

These tests exercise the portable driver, not STM32 peripheral registers or
real sensor timing. They require a native gcc; no board or third-party Python
package is needed. Every scenario runs in a separate process.
"""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include "mpu6000.h"
#include "mpu6000_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t now_us;
static uint32_t lock_depth;
static unsigned selects, deselects, dma_starts, stops, recover_calls;
static unsigned transfers, barriers, drdy_enables;
static bool selected, drdy_enabled, dma_active, allow_dma, allow_recover;
static int fail_read, fail_write, corrupt_read, ignored_write;
static uint8_t registers[128];
static uint8_t *active_rx;
static const uint8_t *active_tx;
static uint16_t active_length;
static uint32_t active_since;
static uint32_t interface_written_at;
static bool interface_written;
static uint32_t register_written_at[128];
static uint8_t register_was_written[128];
static unsigned reset_zero_reads, reset_reads_left;

static MPU6000_Status get_status(void) {
    MPU6000_Status value;
    MPU6000_GetStatus(&value);
    return value;
}

uint32_t MPU6000_Port_NowUs(void) { return now_us; }
uint32_t MPU6000_Port_Lock(void) { return lock_depth++; }
void MPU6000_Port_Unlock(uint32_t key) {
    assert(lock_depth > 0 && key == lock_depth - 1);
    lock_depth = key;
}
void MPU6000_Port_Barrier(void) { ++barriers; }
void MPU6000_Port_Select(bool enable) {
    if (enable) {
        assert(!selected);
        selected = true;
        ++selects;
    } else {
        /* An active transaction must be stopped or complete before CS rises. */
        assert(!dma_active);
        selected = false;
        ++deselects;
    }
}
void MPU6000_Port_EnableDrdy(bool enable) {
    if (enable) {
        /* Publishing RUNNING must precede unmasking the interrupt line. */
        assert(get_status().state == MPU6000_RUNNING);
        assert(registers[0x38] == 1);
        ++drdy_enables;
    }
    drdy_enabled = enable;
}
bool MPU6000_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length) {
    uint8_t reg;
    assert(selected && !dma_active && length == 2);
    ++transfers;
    /* Model a finite transfer duration so reset waits must start afterward. */
    now_us += 24;
    reg = tx[0] & 0x7f;
    rx[0] = rx[1] = 0;
    if (tx[0] & 0x80) {
        if (fail_read == reg) return false;
        rx[1] = registers[reg] ^ (corrupt_read == reg ? 1 : 0);
        /* Model an interface change that is not immediately readable. This is
         * a timing robustness case, not a claim about actual silicon latency. */
        if (reg == 0x6a && interface_written &&
            (uint32_t)(now_us - interface_written_at) < 1000U) rx[1] = 0;
        if (register_was_written[reg] &&
            (uint32_t)(now_us - register_written_at[reg]) < 1000U) rx[1] = 0;
        if (reg == 0x75 && reset_reads_left) { --reset_reads_left; rx[1] = 0; }
    } else {
        if (fail_write == reg) return false;
        if (ignored_write == reg) return true; /* HAL_OK is not a device ACK */
        if (reg == 0x6b && tx[1] == 0x80) {
            uint8_t who = registers[0x75];
            memset(registers, 0, sizeof(registers));
            registers[0x75] = who;
            registers[0x6b] = 0x40;
            interface_written = false;
            memset(register_was_written, 0, sizeof(register_was_written));
            reset_reads_left = reset_zero_reads;
        } else if (reg == 0x68) {
            /* Signal path reset bits self-clear. */
            registers[reg] = 0;
        } else {
            /* Robustness model: the interface selection is ignored in sleep.
             * This is not asserted to be the cause on the user's board. */
            if (reg == 0x6a && (registers[0x6b] & 0x40U)) return true;
            registers[reg] = tx[1];
            register_written_at[reg] = now_us;
            register_was_written[reg] = 1;
            if (reg == 0x6a) {
                interface_written_at = now_us;
                interface_written = true;
            }
        }
    }
    return true;
}
bool MPU6000_Port_TransferRegister(const uint8_t *tx, uint8_t *rx, uint16_t length) {
    bool ok;
    assert(!selected && !dma_active);
    now_us += 10U;
    MPU6000_Port_Select(true);
    ok = MPU6000_Port_Transfer(tx, rx, length);
    MPU6000_Port_Select(false);
    return ok;
}
bool MPU6000_Port_StartDma(const uint8_t *tx, uint8_t *rx, uint16_t length) {
    unsigned i;
    assert(selected && drdy_enabled && !dma_active);
    assert(length == 16 && tx[0] == 0xba);
    for (i = 1; i < length; ++i) assert(tx[i] == 0);
    ++dma_starts;
    if (!allow_dma) return false;
    active_tx = tx;
    active_rx = rx;
    active_length = length;
    active_since = now_us;
    dma_active = true;
    return true;
}
void MPU6000_Port_Stop(void) {
    ++stops;
    dma_active = false;
    drdy_enabled = false;
}
bool MPU6000_Port_Recover(void) {
    ++recover_calls;
    assert(!dma_active && !selected);
    return allow_recover;
}
void MPU6000_Port_RxDmaIRQ(void) { assert(!"board IRQ is outside this test"); }
void MPU6000_Port_Service(void) { }

static void reset_fake(uint32_t initial_us) {
    now_us = initial_us;
    lock_depth = 0;
    selects = deselects = dma_starts = stops = recover_calls = 0;
    transfers = barriers = drdy_enables = 0;
    selected = drdy_enabled = dma_active = false;
    allow_dma = allow_recover = true;
    fail_read = fail_write = corrupt_read = ignored_write = -1;
    active_rx = 0;
    active_tx = 0;
    active_length = 0;
    interface_written = false;
    memset(register_was_written, 0, sizeof(register_was_written));
    reset_zero_reads = reset_reads_left = 0;
    memset(registers, 0, sizeof(registers));
    registers[0x75] = 0x68;
    registers[0x6b] = 0x40;
    MPU6000_Init(now_us);
    assert(get_status().state == MPU6000_POWER_WAIT);
    assert(!selected && !drdy_enabled && lock_depth == 0);
}
static void service_after(uint32_t delta) {
    now_us += delta;
    MPU6000_Service(now_us);
    assert(lock_depth == 0);
}
static void finish_initialization(void) {
    unsigned guard;
    for (guard = 0; guard < 80; ++guard) {
        MPU6000_State state = get_status().state;
        if (state == MPU6000_RUNNING || state == MPU6000_FAULT) return;
        service_after(state == MPU6000_CONFIGURE ? 10000U : 100000U);
    }
    assert(!"initialization failed to make progress");
}
static void boot(uint32_t initial_us) {
    reset_fake(initial_us);
    finish_initialization();
    assert(get_status().state == MPU6000_RUNNING);
    assert(drdy_enabled && !selected && !dma_active);
}
static void start_at(uint32_t time_us) {
    now_us = time_us;
    MPU6000_OnDataReady(now_us);
}
static void be16(uint8_t *out, int16_t value) {
    out[0] = (uint8_t)((uint16_t)value >> 8);
    out[1] = (uint8_t)value;
}
static void complete(unsigned duration_us, uint8_t interrupt_status) {
    assert(dma_active && active_length == 16 && active_rx && selected);
    memset(active_rx, 0, 16);
    active_rx[0] = 0xa7; /* dummy byte must never appear in the sample */
    active_rx[1] = interrupt_status;
    be16(active_rx + 2, -32768);
    be16(active_rx + 4, -2048);
    be16(active_rx + 6, 2048);
    be16(active_rx + 8, -340);
    be16(active_rx + 10, -164);
    be16(active_rx + 12, 0);
    be16(active_rx + 14, 32767);
    now_us = active_since + duration_us;
    dma_active = false;
    MPU6000_OnDmaComplete(now_us);
    assert(!selected && lock_depth == 0);
}
static void valid_at(uint32_t at) { start_at(at); complete(200, 1); }
static void assert_fault(MPU6000_Error error) {
    MPU6000_Status s = get_status();
    assert(s.state == MPU6000_FAULT && s.last_error == error);
    assert(!selected && !drdy_enabled && !dma_active);
    assert(stops == 1 && lock_depth == 0);
}

static void test_initialization(void) {
    MPU6000_Status s;
    unsigned before;
    reset_fake(0);
    MPU6000_OnDataReady(now_us);
    MPU6000_OnDmaComplete(now_us);
    MPU6000_OnSpiError();
    assert(dma_starts == 0 && get_status().drdy_count == 0);
    service_after(99999); assert(transfers == 0);
    service_after(1);
    assert(get_status().state == MPU6000_RESET_WAIT && transfers == 2);
    assert(!drdy_enabled && !selected);
    service_after(99999); assert(transfers == 2);
    service_after(1); assert(get_status().state == MPU6000_SIGNAL_WAIT);
    service_after(99999); assert(get_status().state == MPU6000_SIGNAL_WAIT);
    service_after(1); assert(get_status().state == MPU6000_WAKE_WAIT);
    assert(!interface_written && registers[0x6b] == 1);
    service_after(99999); assert(get_status().state == MPU6000_WAKE_WAIT);
    service_after(1); assert(get_status().state == MPU6000_CONFIGURE);
    assert(!drdy_enabled && registers[0x38] == 0);
    assert(interface_written && registers[0x6a] == 0x10);
    before = transfers;
    service_after(9999); assert(transfers == before && !drdy_enabled && !selected);
    service_after(1); assert(transfers == before + 1); /* verify interface only */
    service_after(1); assert(transfers == before + 2); /* next config write */
    before = transfers;
    service_after(9999); assert(transfers == before && !drdy_enabled && !selected);
    service_after(1); assert(transfers == before + 1); /* separate readback */
    finish_initialization();
    s = get_status();
    assert(s.state == MPU6000_RUNNING && s.who_am_i == 0x68);
    assert(s.last_error == MPU6000_ERROR_NONE && s.bad_register == 0);
    assert(registers[0x6b] == 1 && registers[0x6c] == 0);
    assert(registers[0x6a] == 0x10 && registers[0x1a] == 2);
    assert(registers[0x19] == 0 && registers[0x1b] == 0x18);
    assert(registers[0x1c] == 0x18 && registers[0x23] == 0);
    assert(registers[0x37] == 0 && registers[0x38] == 1);
    assert(drdy_enables == 1 && drdy_enabled && !selected);
    assert(!MPU6000_IsHealthy(now_us)); /* RUNNING alone isn't a valid sample */
}
static void test_id_rejection(void) {
    reset_fake(0); registers[0x75] = 0x70;
    finish_initialization();
    assert_fault(MPU6000_ERROR_ID);
    assert(get_status().who_am_i == 0x70 && transfers == 1);
    service_after(1000000);
    MPU6000_OnDataReady(now_us);
    assert(transfers == 1 && dma_starts == 0);
}
static void test_reset_settling(void) {
    unsigned before;
    reset_fake(0);
    reset_zero_reads = 2;
    service_after(100000); /* identity check then reset */
    service_after(100000); /* transient zero while reset settles */
    assert(get_status().state == MPU6000_RESET_WAIT && get_status().last_error == 0);
    assert(get_status().readback_actual == 0 && dma_starts == 0 && !drdy_enabled);
    before = transfers;
    service_after(9000); assert(transfers == before);
    service_after(1000); assert(get_status().state == MPU6000_RESET_WAIT);
    service_after(10000); assert(get_status().state == MPU6000_SIGNAL_WAIT);
    finish_initialization();
    assert(get_status().state == MPU6000_RUNNING);

    reset_fake(0);
    reset_zero_reads = 10000; /* must stop after the one-second reset deadline */
    finish_initialization();
    assert_fault(MPU6000_ERROR_READBACK);
    assert(get_status().bad_register == 0x75 && get_status().readback_actual == 0);
    assert(dma_starts == 0 && (uint32_t)now_us < 1300000U);
}
static void test_register_io(void) {
    reset_fake(0); fail_read = 0x75;
    finish_initialization(); assert_fault(MPU6000_ERROR_REGISTER_IO);
    assert(get_status().spi_errors == 1);
    reset_fake(0); fail_write = 0x6b;
    finish_initialization(); assert_fault(MPU6000_ERROR_REGISTER_IO);
    assert(get_status().spi_errors == 1);
    reset_fake(0); fail_read = 0x1b;
    finish_initialization(); assert_fault(MPU6000_ERROR_REGISTER_IO);
    assert(get_status().spi_errors == 1 && get_status().bad_register == 0x1b);
    reset_fake(0); fail_write = 0x6a;
    finish_initialization(); assert_fault(MPU6000_ERROR_REGISTER_IO);
    assert(get_status().bad_register == 0x6a && get_status().readback_expected == 0x10);
    reset_fake(0); fail_read = 0x6a;
    finish_initialization(); assert_fault(MPU6000_ERROR_REGISTER_IO);
    assert(get_status().bad_register == 0x6a && get_status().spi_errors == 1);
    reset_fake(0); fail_read = 0x6b;
    finish_initialization(); assert_fault(MPU6000_ERROR_REGISTER_IO);
    assert(get_status().bad_register == 0x6b && !interface_written);
}
static void test_readback(void) {
    reset_fake(0); corrupt_read = 0x1b;
    finish_initialization(); assert_fault(MPU6000_ERROR_READBACK);
    assert(get_status().bad_register == 0x1b);
    assert(get_status().readback_expected == 0x18 && get_status().readback_actual == 0x19);
    assert(get_status().spi_errors == 0 && !drdy_enabled);
    reset_fake(0); corrupt_read = 0x6a;
    finish_initialization(); assert_fault(MPU6000_ERROR_READBACK);
    assert(get_status().bad_register == 0x6a);
    assert(get_status().readback_expected == 0x10 && get_status().readback_actual == 0x11);
    assert(dma_starts == 0 && get_status().spi_errors == 0);
    /* Reproduce the board's wr=10/rd=00 result without weakening the check. */
    reset_fake(0); ignored_write = 0x6a;
    finish_initialization(); assert_fault(MPU6000_ERROR_READBACK);
    assert(get_status().bad_register == 0x6a && get_status().readback_expected == 0x10);
    assert(get_status().readback_actual == 0 && dma_starts == 0);
    reset_fake(0); corrupt_read = 0x6b;
    finish_initialization(); assert_fault(MPU6000_ERROR_READBACK);
    assert(get_status().readback_expected == 0x40 && get_status().readback_actual == 0x41);
    assert(!interface_written && !drdy_enabled);
}
static void test_raw_conversion(void) {
    MPU6000_Sample sample;
    MPU6000_Measurement value;
    uint32_t first;
    boot(0); first = now_us + 1000;
    assert(!MPU6000_PopSample(&sample) && !MPU6000_PopSample(0));
    valid_at(first);
    assert(MPU6000_PopSample(&sample));
    assert(sample.timestamp_us == first && sample.sequence == 1);
    assert(sample.dt_us == 0 && sample.flags == MPU6000_SAMPLE_FIRST);
    assert(sample.accel_raw[0] == -32768 && sample.accel_raw[1] == -2048);
    assert(sample.accel_raw[2] == 2048 && sample.temperature_raw == -340);
    assert(sample.gyro_raw[0] == -164 && sample.gyro_raw[1] == 0);
    assert(sample.gyro_raw[2] == 32767);
    MPU6000_Convert(&sample, &value);
    assert(fabsf(value.accel_m_s2[0] + 16.0f * 9.80665f) < 0.0001f);
    assert(fabsf(value.accel_m_s2[1] + 9.80665f) < 0.00001f);
    assert(fabsf(value.accel_m_s2[2] - 9.80665f) < 0.00001f);
    assert(fabsf(value.gyro_rad_s[0] + 0.1745329252f) < 0.00001f);
    assert(value.gyro_rad_s[1] == 0.0f);
    assert(fabsf(value.gyro_rad_s[2] - (32767.0f / 16.4f) * 0.01745329252f) < 0.0001f);
    assert(fabsf(value.temperature_c - 35.53f) < 0.00001f);
    assert(MPU6000_IsHealthy(now_us));
    assert(get_status().max_transfer_us == 200);
    valid_at(first + 1000);
    assert(MPU6000_PopSample(&sample));
    assert(sample.sequence == 2 && sample.dt_us == 1000 && sample.flags == 0);
    assert(!MPU6000_PopSample(&sample) && barriers >= 4);
}
static void test_cs_and_busy_drop(void) {
    MPU6000_Sample sample;
    uint8_t tx_copy[16];
    const uint8_t *first_tx;
    uint8_t *first_rx;
    uint32_t first;
    unsigned original_selects;
    boot(0); first = now_us + 1000;
    start_at(first);
    assert(selected && dma_active && dma_starts == 1);
    first_tx = active_tx; first_rx = active_rx;
    memcpy(tx_copy, active_tx, 16);
    original_selects = selects;
    start_at(first + 100);
    assert(selected && dma_active && dma_starts == 1 && selects == original_selects);
    assert(active_tx == first_tx && active_rx == first_rx);
    assert(memcmp(tx_copy, active_tx, 16) == 0);
    assert(!MPU6000_PopSample(&sample));
    assert(get_status().busy_drops == 1 && get_status().drdy_count == 2);
    complete(200, 1);
    assert(MPU6000_PopSample(&sample) && sample.timestamp_us == first);
    assert(sample.sequence == 1);
    MPU6000_OnDmaComplete(now_us);
    assert(get_status().samples == 1 && !MPU6000_PopSample(&sample));
    valid_at(first + 2000);
    assert(MPU6000_PopSample(&sample));
    assert(sample.sequence == 3 && sample.dt_us == 2000);
    assert(sample.flags == MPU6000_SAMPLE_GAP);
}
static void test_queue_overflow(void) {
    MPU6000_Sample sample;
    MPU6000_Status s;
    uint32_t first;
    unsigned i;
    boot(0); first = now_us + 1000;
    for (i = 0; i < MPU6000_QUEUE_CAPACITY + 4; ++i) valid_at(first + i * 1000);
    s = get_status();
    assert(s.samples == MPU6000_QUEUE_CAPACITY + 4 && s.queue_drops == 4);
    assert(s.busy_drops == 0 && s.state == MPU6000_RUNNING);
    for (i = 0; i < MPU6000_QUEUE_CAPACITY; ++i) {
        assert(MPU6000_PopSample(&sample));
        assert(sample.sequence == i + 1 && sample.timestamp_us == first + i * 1000);
    }
    assert(!MPU6000_PopSample(&sample));
    valid_at(first + (MPU6000_QUEUE_CAPACITY + 4) * 1000);
    assert(MPU6000_PopSample(&sample));
    assert(sample.sequence == MPU6000_QUEUE_CAPACITY + 5);
    assert(get_status().queue_drops == 4);
}
static void test_drdy_timeout(void) {
    MPU6000_Sample sample;
    uint32_t first;
    boot(0);
    service_after(5000); assert(get_status().state == MPU6000_RUNNING);
    service_after(1); assert_fault(MPU6000_ERROR_DRDY_TIMEOUT);
    assert(get_status().timeouts == 1);
    boot(0); first = now_us + 1000; valid_at(first);
    now_us = first + 5000;
    assert(MPU6000_IsHealthy(now_us));
    MPU6000_Service(now_us); assert(get_status().state == MPU6000_RUNNING);
    service_after(1); assert_fault(MPU6000_ERROR_DRDY_TIMEOUT);
    assert(!MPU6000_IsHealthy(now_us));
    assert(!MPU6000_PopSample(&sample)); /* fault flushes an older queued sample */
}
static void test_dma_timeout(void) {
    MPU6000_Sample sample;
    uint32_t first;
    boot(0); first = now_us + 1000;
    start_at(first);
    service_after(500); assert(get_status().state == MPU6000_RUNNING && selected);
    service_after(1); assert_fault(MPU6000_ERROR_DMA_TIMEOUT);
    assert(get_status().timeouts == 1);
    MPU6000_OnDmaComplete(now_us);
    assert(!MPU6000_PopSample(&sample) && get_status().samples == 0);
}
static void test_late_dma_completion(void) {
    MPU6000_Sample sample;
    boot(0); start_at(now_us + 1000); complete(501, 1);
    assert_fault(MPU6000_ERROR_DMA_TIMEOUT);
    assert(get_status().max_transfer_us == 501 && get_status().timeouts == 1);
    assert(!MPU6000_PopSample(&sample));
}
static void test_invalid_data_status(void) {
    MPU6000_Sample sample;
    uint32_t first;
    boot(0); first = now_us + 1000;
    start_at(first); complete(200, 0);
    assert(get_status().invalid_samples == 1 && !MPU6000_PopSample(&sample));
    start_at(first + 1000); complete(200, 0xff);
    assert(get_status().invalid_samples == 2 && get_status().state == MPU6000_RUNNING);
    valid_at(first + 2000); /* a valid sample resets the invalid streak */
    start_at(first + 3000); complete(200, 0);
    start_at(first + 4000); complete(200, 0);
    assert(get_status().state == MPU6000_RUNNING);
    start_at(first + 5000); complete(200, 0);
    assert_fault(MPU6000_ERROR_DATA_STATUS);
    assert(get_status().invalid_samples == 5 && get_status().samples == 1);
    assert(!MPU6000_PopSample(&sample));
}
static void test_spi_fault_late_callbacks(void) {
    MPU6000_Sample sample;
    MPU6000_Status before, after;
    uint32_t first;
    boot(0); first = now_us + 1000;
    valid_at(first); /* leave a queued sample */
    start_at(first + 1000);
    MPU6000_OnSpiError();
    assert_fault(MPU6000_ERROR_SPI);
    before = get_status();
    assert(before.spi_errors == 1 && !MPU6000_PopSample(&sample));
    MPU6000_OnDmaComplete(now_us + 200);
    MPU6000_OnDataReady(now_us + 1000);
    MPU6000_OnSpiError();
    service_after(1000000);
    after = get_status();
    assert(after.samples == before.samples && after.drdy_count == before.drdy_count);
    assert(after.spi_errors == 1 && stops == 1 && dma_starts == 2);
    assert(!MPU6000_IsHealthy(now_us) && after.state == MPU6000_FAULT);
}
static void test_dma_start_failure(void) {
    boot(0); allow_dma = false;
    start_at(now_us + 1000);
    assert_fault(MPU6000_ERROR_DMA_START);
    assert(get_status().spi_errors == 1 && dma_starts == 1);
}
static void test_clock_wraparound(void) {
    MPU6000_Sample sample;
    uint32_t first = 0xffffff90U;
    boot(0xfffffff0U); /* startup waits also cross the 32-bit wrap */
    valid_at(first);
    assert(MPU6000_PopSample(&sample));
    assert(sample.timestamp_us == first && get_status().max_transfer_us == 200);
    assert(MPU6000_IsHealthy(now_us));
    valid_at(first + 1000U);
    assert(MPU6000_PopSample(&sample));
    assert(sample.dt_us == 1000 && sample.flags == 0);
    service_after(1); assert(get_status().state == MPU6000_RUNNING);
    /* Exercise the foreground timeout subtraction across wrap as well. */
    boot(0); start_at(0xffffff90U);
    service_after(501); assert_fault(MPU6000_ERROR_DMA_TIMEOUT);
}
static void test_stale_caller_timestamp(void) {
    uint32_t stale, first;
    boot(0); first = now_us + 1000; stale = first - 1;
    valid_at(first);
    /* Main's captured timestamp can precede a just-completed ISR sample. */
    MPU6000_Service(stale);
    assert(get_status().state == MPU6000_RUNNING);
    assert(MPU6000_IsHealthy(stale));
    assert(get_status().timeouts == 0);
}
static void test_explicit_recovery(void) {
    MPU6000_Sample sample;
    uint32_t first;
    boot(0);
    assert(!MPU6000_RestartDisarmed(now_us) && recover_calls == 0);
    valid_at(now_us + 1000);
    MPU6000_OnSpiError(); assert_fault(MPU6000_ERROR_SPI);
    allow_recover = false;
    assert(!MPU6000_RestartDisarmed(now_us));
    assert(get_status().state == MPU6000_FAULT && recover_calls == 1);
    allow_recover = true;
    assert(MPU6000_RestartDisarmed(now_us));
    assert(get_status().state == MPU6000_POWER_WAIT && recover_calls == 2);
    assert(!drdy_enabled && !selected);
    assert(!MPU6000_PopSample(&sample));
    assert(get_status().spi_errors == 0 && get_status().samples == 0);
    MPU6000_OnDmaComplete(now_us); /* late callback during startup is ignored */
    finish_initialization();
    assert(get_status().state == MPU6000_RUNNING);
    first = now_us + 1000; valid_at(first);
    assert(MPU6000_PopSample(&sample));
    assert(sample.sequence == 1 && sample.dt_us == 0 && sample.flags == MPU6000_SAMPLE_FIRST);
    assert(MPU6000_IsHealthy(now_us));
}

typedef void (*TestFunction)(void);
typedef struct { const char *name; TestFunction run; } Test;
int main(int argc, char **argv) {
    unsigned i;
    static const Test tests[] = {
        {"initialization", test_initialization},
        {"reset_settling", test_reset_settling},
        {"id_rejection", test_id_rejection},
        {"register_io", test_register_io},
        {"readback", test_readback},
        {"raw_conversion", test_raw_conversion},
        {"cs_and_busy_drop", test_cs_and_busy_drop},
        {"queue_overflow", test_queue_overflow},
        {"drdy_timeout", test_drdy_timeout},
        {"dma_timeout", test_dma_timeout},
        {"late_dma_completion", test_late_dma_completion},
        {"invalid_data_status", test_invalid_data_status},
        {"spi_fault_late_callbacks", test_spi_fault_late_callbacks},
        {"dma_start_failure", test_dma_start_failure},
        {"clock_wraparound", test_clock_wraparound},
        {"stale_caller_timestamp", test_stale_caller_timestamp},
        {"explicit_recovery", test_explicit_recovery}
    };
    assert(argc == 2);
    for (i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (strcmp(argv[1], tests[i].name) == 0) {
            tests[i].run();
            assert(lock_depth == 0);
            puts(tests[i].name);
            return 0;
        }
    }
    fprintf(stderr, "unknown scenario: %s\n", argv[1]);
    return 2;
}
'''


@unittest.skipUnless(shutil.which("gcc"), "native gcc is needed for MPU6000 driver tests")
class Mpu6000DriverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        directory = Path(cls.temporary.name)
        harness = directory / "test_mpu6000.c"
        harness.write_text(HARNESS, encoding="utf-8")
        cls.executable = directory / "test_mpu6000.exe"
        build = subprocess.run(
            [shutil.which("gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-I", str(PROJECT / "Core" / "Inc"),
             str(PROJECT / "Core" / "Src" / "mpu6000.c"), str(harness),
             "-lm", "-o", str(cls.executable)],
            capture_output=True, text=True, timeout=30,
        )
        if build.returncode:
            raise AssertionError(build.stdout + build.stderr)

    def run_scenario(self, name):
        result = subprocess.run([str(self.executable), name], capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), name)

    def test_initialization(self):
        self.run_scenario("initialization")

    def test_reset_settling(self):
        self.run_scenario("reset_settling")

    def test_id_rejection(self):
        self.run_scenario("id_rejection")

    def test_register_io(self):
        self.run_scenario("register_io")

    def test_readback(self):
        self.run_scenario("readback")

    def test_raw_conversion(self):
        self.run_scenario("raw_conversion")

    def test_cs_and_busy_drop(self):
        self.run_scenario("cs_and_busy_drop")

    def test_queue_overflow(self):
        self.run_scenario("queue_overflow")

    def test_drdy_timeout(self):
        self.run_scenario("drdy_timeout")

    def test_dma_timeout(self):
        self.run_scenario("dma_timeout")

    def test_late_dma_completion(self):
        self.run_scenario("late_dma_completion")

    def test_invalid_data_status(self):
        self.run_scenario("invalid_data_status")

    def test_spi_fault_late_callbacks(self):
        self.run_scenario("spi_fault_late_callbacks")

    def test_dma_start_failure(self):
        self.run_scenario("dma_start_failure")

    def test_clock_wraparound(self):
        self.run_scenario("clock_wraparound")

    def test_stale_caller_timestamp(self):
        self.run_scenario("stale_caller_timestamp")

    def test_explicit_recovery(self):
        self.run_scenario("explicit_recovery")


if __name__ == "__main__":
    unittest.main()
