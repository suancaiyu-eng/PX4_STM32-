#include "imu_telemetry.h"
#include <stddef.h>
#include <string.h>
#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
#include <stdio.h>
#endif

#define TELEMETRY_IRQ_PRIORITY 8U

typedef enum {
    TX_IDLE = 0,
    TX_ACTIVE,
    TX_ABORTING,
    TX_FAULT
} TxState;

static UART_HandleTypeDef *telemetry_uart;
/* Ordinary static SRAM; the linker must not place this object in CCM. */
static uint8_t tx_frame[IMU_TELEMETRY_TX_BUFFER_SIZE];
static volatile TxState tx_state;
static volatile uint8_t tx_error_pending;
static volatile ImuTelemetryStats counters;
static uint32_t last_slot_us;
static uint32_t tx_started_us;
static uint32_t abort_started_us;
static uint32_t frame_sequence;
static uint8_t rc_only;
#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
static char text_queue[4][IMU_TELEMETRY_TX_BUFFER_SIZE];
static uint16_t text_lengths[4];
static uint32_t text_write, text_read;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
static uint32_t last_status_us;
static uint8_t have_status;
static uint32_t last_attitude_status_us;
static uint8_t have_attitude_status;
#endif
#endif

/* Mask only priority 8 and lower. Priority-5 IMU IRQs remain enabled.
 * No polling HAL operation may be placed inside these short sections.
 */
static uint32_t lock_uart_irqs(void)
{
    uint32_t previous = __get_BASEPRI();
    __set_BASEPRI_MAX(TELEMETRY_IRQ_PRIORITY << (8U - __NVIC_PRIO_BITS));
    __DSB();
    __ISB();
    return previous;
}

static void unlock_uart_irqs(uint32_t previous)
{
    __DMB();
    __set_BASEPRI(previous);
}

#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_BINARY
static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8U);
    dst[2] = (uint8_t)(value >> 16U);
    dst[3] = (uint8_t)(value >> 24U);
}

static uint16_t crc16(const uint8_t *data, uint32_t size)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint32_t bit;
    for (i = 0U; i < size; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 0x8000U) != 0U
                ? ((uint32_t)crc << 1U) ^ 0x1021U : (uint32_t)crc << 1U);
        }
    }
    return crc;
}

static void encode_frame(const ImuTelemetryRecord *record, uint32_t now_us)
{
    uint32_t axis;
    tx_frame[0] = 0xA5U;
    tx_frame[1] = 0x5AU;
    tx_frame[2] = 1U;
    tx_frame[3] = IMU_TELEMETRY_FRAME_SIZE;
    put_u32(&tx_frame[4], frame_sequence);
    put_u32(&tx_frame[8], record->sequence);
    put_u32(&tx_frame[12], record->timestamp_us);
    put_u32(&tx_frame[16], now_us);
    for (axis = 0U; axis < 3U; ++axis) {
        put_u16(&tx_frame[20U + 2U * axis], (uint16_t)record->accel_raw[axis]);
        put_u16(&tx_frame[26U + 2U * axis], (uint16_t)record->gyro_raw[axis]);
    }
    put_u16(&tx_frame[32], (uint16_t)record->temperature_raw);
    put_u16(&tx_frame[34], record->flags);
    tx_frame[36] = record->driver_state;
    tx_frame[37] = record->driver_error;
    put_u32(&tx_frame[38], record->sample_drops);
    put_u32(&tx_frame[42], record->spi_errors);
    put_u32(&tx_frame[46], record->timeouts);
    put_u32(&tx_frame[50], counters.frames_dropped);
    put_u32(&tx_frame[54], counters.uart_errors);
    put_u32(&tx_frame[58], counters.tx_timeouts);
    put_u16(&tx_frame[62], crc16(&tx_frame[2], 60U));
}
#elif IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
static uint16_t encode_firewater(const ImuTelemetryRecord *record)
{
    int length;
    /* Formatting is foreground-only. No floating point, allocation or printf
     * retargeting; DMA retains ownership of this static buffer until completion.
     * Worst case: 8 prefix + 7*6 signed digits + 6 commas + LF = 57 bytes.
     * The trailing NUL is for snprintf only and is never sent on the wire. */
    length = snprintf((char *)tx_frame, sizeof(tx_frame),
                      "mpu6000:%d,%d,%d,%d,%d,%d,%d\n",
                      (int)record->accel_raw[0], (int)record->accel_raw[1],
                      (int)record->accel_raw[2], (int)record->gyro_raw[0],
                      (int)record->gyro_raw[1], (int)record->gyro_raw[2],
                      (int)record->temperature_raw);
    if (length <= 0 || (size_t)length >= sizeof(tx_frame)) { return 0U; }
    return (uint16_t)length;
}

static uint16_t append_attitude(const ImuTelemetryRecord *record, uint32_t now_us,
                               uint16_t length, uint8_t *attached)
{
    int added;
    uint32_t i;
    uint16_t numeric_length;
    uint8_t valid;
    char angle[3][16];
    char field[3][16];
    *attached = 0U;
    if (record->attitude_present == 0U) { return length; }
    valid = (uint8_t)((record->attitude_flags & 1U) != 0U &&
                     (uint32_t)(now_us - record->attitude_timestamp_us) <= 10000U);
    for (i = 0U; i < 3U; ++i) {
        if (record->attitude_mdeg[i] < -180000 || record->attitude_mdeg[i] > 180000) { valid = 0U; }
    }
    for (i = 0U; i < 3U; ++i) {
            int32_t milli = valid ? record->attitude_mdeg[i] : 0;
            uint32_t magnitude;
            magnitude = (uint32_t)(milli < 0 ? -milli : milli);
            added = snprintf(angle[i], sizeof(angle[i]), "%s%lu.%03lu",
                             milli < 0 ? "-" : "", (unsigned long)(magnitude/1000U),
                             (unsigned long)(magnitude%1000U));
            if (added <= 0 || (size_t)added >= sizeof(angle[i])) { return length; }
    }
    /* FireWater prefixes do NOT select separate channel groups. Use one fixed
     * 11-channel numeric line, preserving the original raw values in CH0..6.
     * Invalid angles are explicit zero placeholders with CH10=0, never held
     * valid-looking stale angles. Raw-only records retain the original 7 fields. */
    if(record->mag_present) {
        for(i=0U;i<3U;++i) {
            int32_t milli=record->mag_healthy ? record->mag_raw_mut[i] : 0;
            uint32_t magnitude;
            if(milli < -1000000 || milli > 1000000) { milli=0; }
            magnitude=(uint32_t)(milli < 0 ? -milli : milli);
            added=snprintf(field[i],sizeof(field[i]),"%s%lu.%03lu",milli < 0 ? "-" : "",
                           (unsigned long)(magnitude/1000U),(unsigned long)(magnitude%1000U));
            if(added<=0 || (size_t)added>=sizeof(field[i])) { return length; }
        }
        added=snprintf((char *)&tx_frame[length-1U],sizeof(tx_frame)-length+1U,
            ",%s,%s,%s,%u,%s,%s,%s,%u,%u,%u,%u\n",angle[0],angle[1],angle[2],(unsigned int)valid,
            field[0],field[1],field[2],(unsigned int)(record->heading_valid != 0U && valid &&
                record->mag_calibrated && record->mag_used && record->mag_healthy),
            (unsigned int)(record->mag_calibrated != 0U),(unsigned int)(record->mag_used != 0U && valid && record->mag_healthy),
            (unsigned int)(record->mag_healthy != 0U));
    } else {
        added = snprintf((char *)&tx_frame[length - 1U], sizeof(tx_frame) - length + 1U,
                         ",%s,%s,%s,%u\n", angle[0], angle[1], angle[2], (unsigned int)valid);
    }
    if (added <= 0 || (size_t)added >= sizeof(tx_frame) - length + 1U) {
        tx_frame[length - 1U] = '\n'; tx_frame[length] = 0U;
        return length;
    }
    numeric_length = (uint16_t)(length - 1U + added);
    if (valid) { *attached = 1U; return numeric_length; }
    {
        if (have_attitude_status &&
            (uint32_t)(now_us - last_attitude_status_us) < IMU_TELEMETRY_STATUS_PERIOD_US) {
            return numeric_length;
        }
        added = snprintf((char *)&tx_frame[numeric_length], sizeof(tx_frame) - numeric_length,
                         "#ATT state=%lu error=%lu cal=%lu reset=%lu\n",
                         (unsigned long)record->attitude_state,
                         (unsigned long)record->attitude_error,
                         (unsigned long)record->calibration_samples,
                         (unsigned long)record->calibration_restarts);
        *attached = 2U;
    }
    if (added <= 0 || (size_t)added >= sizeof(tx_frame) - numeric_length) {
        /* Keep all numeric channels if an optional diagnostic cannot fit. */
        tx_frame[numeric_length] = 0U;
        *attached = 0U;
        return numeric_length;
    }
    return (uint16_t)(numeric_length + added);
}

static uint16_t encode_status(const ImuTelemetryRecord *record)
{
    /* Distinct from the seven numeric measurement channels; inspect as raw text.
     * A missing/uninitialized IMU must not make a working UART appear dead. */
    int length = snprintf((char *)tx_frame, sizeof(tx_frame),
        "#IMU state=%u error=%u id=0x%02X reg=0x%02X wr=%02X rd=%02X drdy=%lu samples=%lu spi=%lu timeout=%lu\n",
        (unsigned int)record->driver_state, (unsigned int)record->driver_error,
        (unsigned int)record->who_am_i, (unsigned int)record->bad_register,
        (unsigned int)record->readback_expected, (unsigned int)record->readback_actual,
        (unsigned long)record->drdy_count, (unsigned long)record->samples,
        (unsigned long)record->spi_errors, (unsigned long)record->timeouts);
    if (length <= 0 || (size_t)length >= sizeof(tx_frame)) { return 0U; }
    return (uint16_t)length;
}
#else
static uint16_t encode_seekfree(const ImuTelemetryRecord *record, uint32_t now_us)
{
    int length;
#if IMU_TELEMETRY_VIEW <= IMU_TELEMETRY_VIEW_MAG
    const char *description;
    char axis[3][20];
    int32_t milli[3];
    uint32_t i;
    uint8_t valid = (uint8_t)((record->flags & IMU_TELEMETRY_FLAG_SAMPLE_VALID) != 0U &&
        (record->flags & (IMU_TELEMETRY_FLAG_STALE | IMU_TELEMETRY_FLAG_DRIVER_FAULT)) == 0U &&
        (uint32_t)(now_us - record->timestamp_us) <= 5000U);
#if IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_GYRO
    description = "gyro";
    /* MPU6000 +/-2000 deg/s: 16.4 LSB/(deg/s). Sensor-native axes,
     * before estimator bias correction; rounded to 0.001 deg/s. */
    for (i = 0U; i < 3U; ++i) {
        int32_t scaled = (int32_t)record->gyro_raw[i] * 10000;
        milli[i] = (scaled + (scaled < 0 ? -82 : 82)) / 164;
    }
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_ACCEL
    description = "accel";
    /* MPU6000 +/-16 g: 2048 LSB/g. Includes gravity. */
    for (i = 0U; i < 3U; ++i) {
        int32_t scaled = (int32_t)record->accel_raw[i] * 1000;
        milli[i] = (scaled + (scaled < 0 ? -1024 : 1024)) / 2048;
    }
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_ATTITUDE
    description = "attitude";
    valid = (uint8_t)(record->attitude_present && (record->attitude_flags & 1U) != 0U &&
        (uint32_t)(now_us - record->attitude_timestamp_us) <= 10000U);
    for (i = 0U; i < 3U; ++i) {
        milli[i] = record->attitude_mdeg[i];
        if (milli[i] < -180000 || milli[i] > 180000) { valid = 0U; }
    }
#else
    description = "mag";
    valid = (uint8_t)(record->mag_present && record->mag_healthy);
    for (i = 0U; i < 3U; ++i) {
        milli[i] = record->mag_raw_mut[i];
        if (milli[i] < -1000000 || milli[i] > 1000000) { valid = 0U; }
    }
#endif
    /* Missing/stale samples do not become valid-looking zero or held curves. */
    if (!valid) { return 0U; }
    for (i = 0U; i < 3U; ++i) {
        uint32_t magnitude = milli[i] < 0 ? 0U - (uint32_t)milli[i] : (uint32_t)milli[i];
        length = snprintf(axis[i], sizeof(axis[i]), "%s%lu.%03lu",
            milli[i] < 0 ? "-" : "", (unsigned long)(magnitude / 1000U),
            (unsigned long)(magnitude % 1000U));
        if (length <= 0 || (size_t)length >= sizeof(axis[i])) { return 0U; }
    }
#if IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_ATTITUDE
    length = snprintf((char *)tx_frame, sizeof(tx_frame), "%s:%s,%s,%s,%u\n",
        description, axis[0], axis[1], axis[2], (unsigned int)(record->heading_valid != 0U));
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_MAG
    length = snprintf((char *)tx_frame, sizeof(tx_frame), "%s:%s,%s,%s,%u\n",
        description, axis[0], axis[1], axis[2], (unsigned int)(record->mag_calibrated != 0U));
#else
    length = snprintf((char *)tx_frame, sizeof(tx_frame), "%s:%s,%s,%s\n",
        description, axis[0], axis[1], axis[2]);
#endif
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_BARO
    (void)now_us;
    /* Flags: bit 0 raw, bit 1 compensated pressure, bit 2 relative altitude.
     * In particular a zero startup height is not valid unless bit 2 is set. */
    length = snprintf((char *)tx_frame, sizeof(tx_frame), "baro:%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu\n",
        (long)record->baro_pressure_pa, (long)record->baro_temperature_centi_c,
        (long)record->baro_relative_mm, (long)record->baro_filtered_mm,
        (unsigned long)record->baro_flags, (unsigned long)record->baro_model,
        (unsigned long)record->baro_state, (unsigned long)record->baro_error);
#else
    (void)record;
    (void)now_us;
    return 0U; /* RC messages use BoardLinkTelem and the same DMA queue. */
#endif
    return length > 0 && (size_t)length < sizeof(tx_frame) ? (uint16_t)length : 0U;
}
#endif

void ImuTelemetry_Init(UART_HandleTypeDef *uart, uint32_t now_us)
{
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_SEEKFREE
    rc_only = (IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_RC);
#else
    rc_only = IMU_TELEMETRY_RC_ONLY;
#endif
    /* Initial setup only; never reset the module while a transfer owns its RAM. */
    telemetry_uart = uart;
    tx_state = TX_IDLE;
    tx_error_pending = 0U;
    memset((void *)&counters, 0, sizeof(counters));
    last_slot_us = now_us;
    tx_started_us = now_us;
    abort_started_us = now_us;
    frame_sequence = 0U;
#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
    text_write = text_read = 0U;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
    last_status_us = now_us;
    have_status = 0U;
    last_attitude_status_us = now_us;
    have_attitude_status = 0U;
#endif
#endif
    if ((uart == NULL) || (uart->hdmatx == NULL)) {
        tx_state = TX_FAULT;
    }
}

uint8_t ImuTelemetry_QueueText(const char *text, uint16_t length)
{
#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
    if (text == NULL || length == 0U || length >= IMU_TELEMETRY_TX_BUFFER_SIZE ||
        text[length - 1U] != '\n' || text_write - text_read >= 4U) {
        ++counters.text_dropped;
        return 0U;
    }
    memcpy(text_queue[text_write % 4U], text, length);
    text_lengths[text_write % 4U] = length;
    text_write++;
    ++counters.text_queued;
    return 1U;
#else
    (void)text;
    (void)length;
    return 0U;
#endif
}

#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
static void send_queued_text(uint32_t now_us)
{
    uint32_t previous;
    uint32_t baud;
    uint32_t needed_us;
    uint32_t remaining_us;
    HAL_StatusTypeDef status;
    uint16_t length;
    if (tx_state != TX_IDLE || text_write == text_read) { return; }
    length = text_lengths[text_read % 4U];
    /* Reserve the next IMU slot. At 8N1 each byte occupies ten wire bits;
     * leave 300 us for main-loop/IRQ scheduling before the next numeric line.
     */
    baud = telemetry_uart->Init.BaudRate;
    if (baud == 0U) { return; }
    needed_us = ((uint32_t)length * 10000000U + baud - 1U) / baud + 300U;
    remaining_us = IMU_TELEMETRY_PERIOD_US - (uint32_t)(now_us - last_slot_us);
    if (!rc_only && needed_us > remaining_us) { return; }
    memcpy(tx_frame, text_queue[text_read % 4U], length);
    previous = lock_uart_irqs();
    tx_error_pending = 0U;
    tx_started_us = now_us;
    tx_state = TX_ACTIVE;
    __DMB();
    status = HAL_UART_Transmit_DMA(telemetry_uart, tx_frame, length);
    if (status == HAL_OK) {
        text_read++;
        ++counters.text_started;
        ++counters.frames_started;
    } else {
        ++counters.start_errors;
        counters.last_hal_error = telemetry_uart->ErrorCode;
        tx_state = ((telemetry_uart->hdmatx->Instance->CR & DMA_SxCR_EN) == 0U)
            ? TX_IDLE : TX_FAULT;
    }
    unlock_uart_irqs(previous);
}
#endif

static void service_recovery(uint32_t now_us)
{
    uint32_t previous = lock_uart_irqs();
    if (tx_state == TX_ACTIVE) {
        uint8_t timed_out = (uint8_t)((uint32_t)(now_us - tx_started_us)
                                     >= IMU_TELEMETRY_TX_TIMEOUT_US);
        if ((tx_error_pending != 0U) || (timed_out != 0U)) {
            if (timed_out != 0U) {
                ++counters.tx_timeouts;
            }
            tx_state = TX_ABORTING;
            abort_started_us = now_us;
            ++counters.aborts;
            /* This HAL version's _IT abort only disables the DMA stream; it
             * never waits for DMA EN or UART TC. A callback may run inline.
             * Holding the UART-level mask prevents an old completion racing
             * the HAL handle mutation, while IMU IRQs can still preempt us.
             */
            if (HAL_UART_AbortTransmit_IT(telemetry_uart) != HAL_OK) {
                tx_state = TX_FAULT;
                ++counters.abort_timeouts;
            }
        }
    } else if ((tx_state == TX_ABORTING)
               && ((uint32_t)(now_us - abort_started_us)
                   >= IMU_TELEMETRY_TX_TIMEOUT_US)) {
        /* An unconfirmed abort must not release a potentially live DMA buffer.
         * Latch a telemetry fault; do not reset the shared DMA controller.
         */
        tx_state = TX_FAULT;
        ++counters.abort_timeouts;
    }
    unlock_uart_irqs(previous);
}

void ImuTelemetry_Service(const ImuTelemetryRecord *latest, uint32_t now_us)
{
    uint32_t elapsed;
    uint32_t slots;
    uint32_t previous;
    uint16_t length;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
    uint8_t is_status = 0U;
    uint8_t attached_attitude = 0U;
#endif
    HAL_StatusTypeDef status;

    service_recovery(now_us);
    /* RC-only console: keep the shared DMA/recovery owner, silence all
     * periodic sensor and status encoders below. */
    if (rc_only) {
#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
        send_queued_text(now_us);
#endif
        return;
    }
    elapsed = (uint32_t)(now_us - last_slot_us);
    if (elapsed < IMU_TELEMETRY_PERIOD_US) {
#if IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY
        send_queued_text(now_us);
#endif
        return;
    }
    slots = elapsed / IMU_TELEMETRY_PERIOD_US;
    last_slot_us += slots * IMU_TELEMETRY_PERIOD_US;
    frame_sequence += slots;
    counters.frames_dropped += slots - 1U;

    if ((tx_state != TX_IDLE) || (latest == NULL)) {
        ++counters.frames_dropped;
        return;
    }

    /* Only main writes this buffer, and TX_IDLE confirms DMA relinquished it.
     * No HAL callback can start a new transfer. Encoding needs no IRQ masking.
     */
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_BINARY
    encode_frame(latest, now_us);
    length = IMU_TELEMETRY_FRAME_SIZE;
#elif IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_SEEKFREE
    length = encode_seekfree(latest, now_us);
    if (length == 0U) { ++counters.frames_dropped; return; }
#else
    if ((latest->flags & IMU_TELEMETRY_FLAG_SAMPLE_VALID) == 0U ||
        (latest->flags & (IMU_TELEMETRY_FLAG_STALE | IMU_TELEMETRY_FLAG_DRIVER_FAULT)) != 0U) {
        if (have_status && (uint32_t)(now_us - last_status_us) < IMU_TELEMETRY_STATUS_PERIOD_US) {
            ++counters.frames_dropped;
            return;
        }
        length = encode_status(latest);
        is_status = 1U;
    } else {
        length = encode_firewater(latest);
        if (length != 0U) { length = append_attitude(latest, now_us, length, &attached_attitude); }
    }
    if (length == 0U) { ++counters.frames_dropped; return; }
#endif
    previous = lock_uart_irqs();
    tx_error_pending = 0U;
    tx_started_us = now_us;
    tx_state = TX_ACTIVE;
    __DMB();
    status = HAL_UART_Transmit_DMA(telemetry_uart, tx_frame, length);
    if (status == HAL_OK) {
        ++counters.frames_started;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
        if (is_status) {
            last_status_us = now_us;
            have_status = 1U;
            ++counters.status_frames;
        }
        if (attached_attitude == 1U) { ++counters.attitude_frames; }
        if (attached_attitude == 2U) {
            last_attitude_status_us = now_us;
            have_attitude_status = 1U;
            ++counters.attitude_status_frames;
        }
#endif
    } else {
        ++counters.start_errors;
        ++counters.frames_dropped;
        counters.last_hal_error = telemetry_uart->ErrorCode;
        /* Preserve ownership if hardware is unexpectedly still enabled. */
        tx_state = ((telemetry_uart->hdmatx->Instance->CR & DMA_SxCR_EN) == 0U)
            ? TX_IDLE : TX_FAULT;
    }
    unlock_uart_irqs(previous);
}

void ImuTelemetry_OnTxComplete(UART_HandleTypeDef *uart)
{
    if ((uart == telemetry_uart) && (tx_state == TX_ACTIVE)
        && (tx_error_pending == 0U)) {
        ++counters.frames_completed;
        __DMB();
        tx_state = TX_IDLE;
    }
}

void ImuTelemetry_OnError(UART_HandleTypeDef *uart)
{
    if (uart == telemetry_uart) {
        ++counters.uart_errors;
        counters.last_hal_error = uart->ErrorCode;
        if (tx_state == TX_ACTIVE) {
            tx_error_pending = 1U;
        }
    }
}

void ImuTelemetry_OnAbortComplete(UART_HandleTypeDef *uart)
{
    if ((uart == telemetry_uart) && (tx_state == TX_ABORTING)) {
        if ((uart->hdmatx->Instance->CR & DMA_SxCR_EN) == 0U) {
            tx_error_pending = 0U;
            __DMB();
            tx_state = TX_IDLE;
        } else {
            tx_state = TX_FAULT;
            ++counters.abort_timeouts;
        }
    }
}

void ImuTelemetry_GetStats(ImuTelemetryStats *stats)
{
    uint32_t previous;
    if (stats == NULL) {
        return;
    }
    previous = lock_uart_irqs();
    stats->frames_started = counters.frames_started;
    stats->frames_completed = counters.frames_completed;
    stats->frames_dropped = counters.frames_dropped;
    stats->uart_errors = counters.uart_errors;
    stats->tx_timeouts = counters.tx_timeouts;
    stats->start_errors = counters.start_errors;
    stats->aborts = counters.aborts;
    stats->abort_timeouts = counters.abort_timeouts;
    stats->last_hal_error = counters.last_hal_error;
    stats->status_frames = counters.status_frames;
    stats->busy = (uint8_t)((tx_state == TX_ACTIVE) || (tx_state == TX_ABORTING));
    stats->faulted = (uint8_t)(tx_state == TX_FAULT);
    stats->attitude_frames = counters.attitude_frames;
    stats->attitude_status_frames = counters.attitude_status_frames;
    stats->text_queued = counters.text_queued;
    stats->text_started = counters.text_started;
    stats->text_dropped = counters.text_dropped;
    unlock_uart_irqs(previous);
}
