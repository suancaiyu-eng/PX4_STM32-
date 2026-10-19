#include "mpu6000.h"
#include "mpu6000_port.h"
#include <stddef.h>
#include <string.h>

#define REG_SMPLRT_DIV  0x19U
#define REG_CONFIG      0x1AU
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_FIFO_EN     0x23U
#define REG_INT_PIN_CFG 0x37U
#define REG_INT_ENABLE  0x38U
#define REG_INT_STATUS  0x3AU
#define REG_SIGNAL_RESET 0x68U
#define REG_USER_CTRL   0x6AU
#define REG_PWR1        0x6BU
#define REG_PWR2        0x6CU
#define REG_WHO_AM_I    0x75U
#define STARTUP_WAIT_US 100000U
#define CONFIG_SETTLE_US 10000U
#define RESET_TIMEOUT_US 1000000U
#define TRANSFER_TIMEOUT_US 500U
#define SAMPLE_TIMEOUT_US 5000U
#define INVALID_LIMIT 3U

typedef struct { uint8_t reg; uint8_t value; } RegisterSetting;
static const RegisterSetting settings[] = {
    {REG_PWR2, 0x00U},
    {REG_CONFIG, 0x02U}, {REG_SMPLRT_DIV, 0x00U},
    {REG_GYRO_CONFIG, 0x18U}, {REG_ACCEL_CONFIG, 0x18U},
    {REG_FIFO_EN, 0x00U},
    /* High/push-pull/50us pulse; read INT_STATUS to clear, not arbitrary reads. */
    {REG_INT_PIN_CFG, 0x00U}, {REG_INT_ENABLE, 0x00U}
};

static volatile MPU6000_Status status;
static volatile uint8_t transfer_busy;
static volatile uint32_t queue_head;
static volatile uint32_t queue_tail;
static MPU6000_Sample queue[MPU6000_QUEUE_CAPACITY];
/* All RW/ZI are linked into 0x20000000 SRAM, not CCM. */
static uint8_t dma_tx[MPU6000_BURST_SIZE];
static uint8_t dma_rx[MPU6000_BURST_SIZE];
static uint32_t transfer_timestamp;
static uint32_t transfer_sequence;
static uint32_t state_since;
static uint32_t reset_poll_since;
static uint32_t run_since;
static uint32_t previous_sample_us;
static uint8_t have_previous_sample;
static uint8_t invalid_streak;
static uint8_t config_index;
static uint8_t interface_checked;
static uint8_t config_pending;

static void fault(MPU6000_Error error)
{
    /* Serializes foreground fault transitions against IRQ publication. */
    uint32_t key = MPU6000_Port_Lock();
    if (status.state != MPU6000_FAULT) {
        status.last_error = error;
        status.state = MPU6000_FAULT;
        MPU6000_Port_Stop();
        MPU6000_Port_Select(false);
        transfer_busy = 0U;
        queue_tail = queue_head; /* don't present pre-fault queued samples as fresh */
    }
    MPU6000_Port_Unlock(key);
}

static bool read_register(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80U), 0U};
    uint8_t rx[2] = {0U, 0U};
    bool ok;
    ok = MPU6000_Port_TransferRegister(tx, rx, 2U);
    *value = rx[1];
    return ok;
}

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2];
    bool ok;
    status.bad_register = reg;
    status.readback_expected = value;
    status.readback_actual = 0U;
    ok = MPU6000_Port_TransferRegister(tx, rx, 2U);
    return ok;
}

static bool readback_matches(uint8_t reg, uint8_t value)
{
    uint8_t actual = 0U;
    status.bad_register = reg;
    status.readback_expected = value;
    status.readback_actual = 0U;
    if (!read_register(reg, &actual)) {
        status.spi_errors++;
        fault(MPU6000_ERROR_REGISTER_IO);
        return false;
    }
    status.readback_actual = actual;
    return actual == value;
}

static bool verify_register(uint8_t reg, uint8_t value)
{
    if (readback_matches(reg, value)) { return true; }
    if (status.state != MPU6000_FAULT) { fault(MPU6000_ERROR_READBACK); }
    return false;
}

static bool configuration_write(uint8_t reg, uint8_t value)
{
    if (!write_register(reg, value)) {
        status.spi_errors++;
        fault(MPU6000_ERROR_REGISTER_IO);
        return false;
    }
    state_since = MPU6000_Port_NowUs();
    return true;
}

void MPU6000_Init(uint32_t now_us)
{
    MPU6000_Port_EnableDrdy(false);
    MPU6000_Port_Select(false);
    memset((void *)&status, 0, sizeof(status));
    memset(dma_tx, 0, sizeof(dma_tx));
    dma_tx[0] = REG_INT_STATUS | 0x80U;
    queue_head = 0U;
    queue_tail = 0U;
    transfer_busy = 0U;
    config_index = 0U;
    interface_checked = config_pending = 0U;
    invalid_streak = 0U;
    have_previous_sample = 0U;
    state_since = now_us;
    status.state = MPU6000_POWER_WAIT;
}

void MPU6000_Service(uint32_t now_us)
{
    uint8_t value;
    uint32_t key;
    switch (status.state) {
    case MPU6000_POWER_WAIT:
        if ((uint32_t)(now_us - state_since) < STARTUP_WAIT_US) { return; }
        if (!read_register(REG_WHO_AM_I, &value)) {
            status.spi_errors++;
            fault(MPU6000_ERROR_REGISTER_IO);
            return;
        }
        status.who_am_i = value;
        if (value != 0x68U) { fault(MPU6000_ERROR_ID); return; }
        if (!write_register(REG_PWR1, 0x80U)) {
            status.spi_errors++;
            fault(MPU6000_ERROR_REGISTER_IO);
            return;
        }
        status.state = MPU6000_RESET_WAIT;
        state_since = MPU6000_Port_NowUs();
        reset_poll_since = state_since;
        break;
    case MPU6000_RESET_WAIT:
        if ((uint32_t)(now_us - state_since) < STARTUP_WAIT_US) { return; }
        if ((uint32_t)(now_us - reset_poll_since) < CONFIG_SETTLE_US) { return; }
        reset_poll_since = now_us;
        /* Confirm the reset completed before relying on subsequent writes. */
        if (!readback_matches(REG_WHO_AM_I, 0x68U) ||
            !readback_matches(REG_PWR1, 0x40U)) {
            if (status.state != MPU6000_FAULT &&
                (uint32_t)(now_us - state_since) >= RESET_TIMEOUT_US) {
                fault(MPU6000_ERROR_READBACK);
            }
            return;
        }
        if (!write_register(REG_SIGNAL_RESET, 0x07U)) {
            status.spi_errors++;
            fault(MPU6000_ERROR_REGISTER_IO);
            return;
        }
        status.state = MPU6000_SIGNAL_WAIT;
        state_since = MPU6000_Port_NowUs();
        break;
    case MPU6000_SIGNAL_WAIT:
        if ((uint32_t)(now_us - state_since) < STARTUP_WAIT_US) { return; }
        /* Wake first; verify the PLL/sleep setting after the wake-up wait. */
        if (!configuration_write(REG_PWR1, 0x01U)) { return; }
        status.state = MPU6000_WAKE_WAIT;
        state_since = MPU6000_Port_NowUs();
        break;
    case MPU6000_WAKE_WAIT:
        if ((uint32_t)(now_us - state_since) < STARTUP_WAIT_US) { return; }
        if (!verify_register(REG_PWR1, 0x01U) ||
            !configuration_write(REG_USER_CTRL, 0x10U)) { return; }
        status.state = MPU6000_CONFIGURE;
        break;
    case MPU6000_CONFIGURE:
        /* Bounded foreground steps: CS is released during every wait. Unlike
         * the reference polling driver, no HAL_Delay or ISR wait is needed. */
        if (!interface_checked) {
            if ((uint32_t)(now_us - state_since) < CONFIG_SETTLE_US) { return; }
            if (!verify_register(REG_USER_CTRL, 0x10U)) { return; }
            interface_checked = 1U;
            return;
        }
        if (config_index < sizeof(settings) / sizeof(settings[0])) {
            if (!config_pending) {
                if (configuration_write(settings[config_index].reg, settings[config_index].value)) {
                    config_pending = 1U;
                }
                return;
            }
            if ((uint32_t)(now_us - state_since) < CONFIG_SETTLE_US) { return; }
            if (verify_register(settings[config_index].reg, settings[config_index].value)) {
                config_index++;
                config_pending = 0U;
            }
            return;
        }
        if (!config_pending) {
            if (!read_register(REG_INT_STATUS, &value)) {
                status.bad_register = REG_INT_STATUS;
                status.readback_expected = status.readback_actual = 0U;
                status.spi_errors++;
                fault(MPU6000_ERROR_REGISTER_IO);
                return;
            }
            if (configuration_write(REG_INT_ENABLE, 0x01U)) {
                config_pending = 1U;
            }
            return;
        }
        if ((uint32_t)(now_us - state_since) < CONFIG_SETTLE_US) { return; }
        if (!verify_register(REG_INT_ENABLE, 0x01U)) { return; }
        /* Driver state must be ready before the EXTI line is unmasked. */
        run_since = MPU6000_Port_NowUs();
        status.bad_register = 0U;
        status.readback_expected = status.readback_actual = 0U;
        status.state = MPU6000_RUNNING;
        MPU6000_Port_EnableDrdy(true);
        break;
    case MPU6000_RUNNING:
        /* Very short critical section, no HAL wait/copy/formatting. */
        key = MPU6000_Port_Lock();
        if (status.state == MPU6000_RUNNING) {
            /* Refresh now after masking IRQs: a just-completed sample may be newer
             * than the caller's now_us, which would otherwise look like wraparound. */
            now_us = MPU6000_Port_NowUs();
            if (transfer_busy && (uint32_t)(now_us - transfer_timestamp) > TRANSFER_TIMEOUT_US) {
                status.timeouts++;
                fault(MPU6000_ERROR_DMA_TIMEOUT);
            } else if ((uint32_t)(now_us - (status.samples ? status.last_sample_us : run_since))
                       > SAMPLE_TIMEOUT_US) {
                status.timeouts++;
                fault(MPU6000_ERROR_DRDY_TIMEOUT);
            }
        }
        MPU6000_Port_Unlock(key);
        break;
    default:
        break;
    }
}

void MPU6000_OnDataReady(uint32_t now_us)
{
    if (status.state != MPU6000_RUNNING) { return; }
    status.drdy_count++;
    if (transfer_busy) { status.busy_drops++; return; }
    transfer_busy = 1U;
    transfer_timestamp = now_us;
    transfer_sequence = status.drdy_count;
    MPU6000_Port_Select(true);
    if (!MPU6000_Port_StartDma(dma_tx, dma_rx, sizeof(dma_tx))) {
        status.spi_errors++;
        fault(MPU6000_ERROR_DMA_START);
    }
}

static int16_t signed_be(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0] << 8U) | p[1];
    return (int16_t)((v & 0x8000U) ? ((int32_t)v - 65536) : (int32_t)v);
}

void MPU6000_OnDmaComplete(uint32_t now_us)
{
    MPU6000_Sample sample;
    uint32_t duration;
    uint32_t head;
    unsigned int axis;
    if (status.state != MPU6000_RUNNING || !transfer_busy) { return; }
    MPU6000_Port_Select(false);
    transfer_busy = 0U;
    duration = (uint32_t)(now_us - transfer_timestamp);
    if (duration > status.max_transfer_us) { status.max_transfer_us = duration; }
    if (duration > TRANSFER_TIMEOUT_US) {
        status.timeouts++;
        fault(MPU6000_ERROR_DMA_TIMEOUT);
        return;
    }
    /* Only DATA_RDY is expected: FIFO/DMP/I2C master are disabled. Reject a
     * floating/stuck SPI line or a stale interrupt rather than publish fake data. */
    if (dma_rx[1] != 0x01U) {
        status.invalid_samples++;
        invalid_streak++;
        if (invalid_streak >= INVALID_LIMIT) { fault(MPU6000_ERROR_DATA_STATUS); }
        return;
    }
    invalid_streak = 0U;
    sample.timestamp_us = transfer_timestamp;
    sample.sequence = transfer_sequence;
    sample.dt_us = have_previous_sample ? (uint32_t)(transfer_timestamp - previous_sample_us) : 0U;
    sample.flags = have_previous_sample ? 0U : MPU6000_SAMPLE_FIRST;
    if (have_previous_sample && (sample.dt_us < 500U || sample.dt_us > 1500U)) {
        sample.flags |= MPU6000_SAMPLE_GAP;
    }
    for (axis = 0U; axis < 3U; axis++) {
        sample.accel_raw[axis] = signed_be(&dma_rx[2U + axis * 2U]);
        sample.gyro_raw[axis] = signed_be(&dma_rx[10U + axis * 2U]);
    }
    sample.temperature_raw = signed_be(&dma_rx[8]);
    previous_sample_us = transfer_timestamp;
    have_previous_sample = 1U;
    status.samples++;
    status.last_sample_us = transfer_timestamp;
    head = queue_head;
    if ((uint32_t)(head - queue_tail) >= MPU6000_QUEUE_CAPACITY) {
        status.queue_drops++;
        return;
    }
    queue[head % MPU6000_QUEUE_CAPACITY] = sample;
    MPU6000_Port_Barrier();
    queue_head = head + 1U;
}

void MPU6000_OnSpiError(void)
{
    if (status.state == MPU6000_RUNNING) {
        status.spi_errors++;
        fault(MPU6000_ERROR_SPI);
    }
}

bool MPU6000_PopSample(MPU6000_Sample *sample)
{
    uint32_t key;
    uint32_t tail;
    if (sample == NULL) { return false; }
    /* Also protects against a fault flushing the queue during a foreground copy. */
    key = MPU6000_Port_Lock();
    tail = queue_tail;
    if (tail == queue_head || status.state != MPU6000_RUNNING) {
        MPU6000_Port_Unlock(key);
        return false;
    }
    MPU6000_Port_Barrier();
    *sample = queue[tail % MPU6000_QUEUE_CAPACITY];
    queue_tail = tail + 1U;
    MPU6000_Port_Unlock(key);
    return true;
}

void MPU6000_Convert(const MPU6000_Sample *sample, MPU6000_Measurement *measurement)
{
    unsigned int axis;
    for (axis = 0U; axis < 3U; axis++) {
        measurement->accel_m_s2[axis] = (float)sample->accel_raw[axis] * (9.80665f / 2048.0f);
        measurement->gyro_rad_s[axis] = (float)sample->gyro_raw[axis] * (0.0174532925199433f / 16.4f);
    }
    measurement->temperature_c = (float)sample->temperature_raw / 340.0f + 36.53f;
}

void MPU6000_GetStatus(MPU6000_Status *out)
{
    uint32_t key = MPU6000_Port_Lock();
    *out = status;
    MPU6000_Port_Unlock(key);
}

bool MPU6000_IsHealthy(uint32_t now_us)
{
    bool healthy;
    uint32_t key = MPU6000_Port_Lock();
    (void)now_us;
    now_us = MPU6000_Port_NowUs();
    healthy = (status.state == MPU6000_RUNNING && status.samples != 0U &&
               (uint32_t)(now_us - status.last_sample_us) <= SAMPLE_TIMEOUT_US);
    MPU6000_Port_Unlock(key);
    return healthy;
}

bool MPU6000_RestartDisarmed(uint32_t now_us)
{
    if (status.state != MPU6000_FAULT || !MPU6000_Port_Recover()) { return false; }
    MPU6000_Init(now_us);
    return true;
}
