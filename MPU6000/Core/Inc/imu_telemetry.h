#ifndef IMU_TELEMETRY_H
#define IMU_TELEMETRY_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_TELEMETRY_PERIOD_US       10000U
#define IMU_TELEMETRY_TX_TIMEOUT_US   20000U
#define IMU_TELEMETRY_FRAME_SIZE      64U /* original binary protocol */
#define IMU_TELEMETRY_TX_BUFFER_SIZE  192U
#define IMU_TELEMETRY_STATUS_PERIOD_US 1000000U

/* Select ONE wire format, then rebuild. Both use the same DMA owner.
 * FireWater raw-only: mpu6000:ax,ay,az,gx,gy,gz,temp\n (int16 raw counts).
 * The flight application defaults to 18 channels: raw IMU, attitude, raw field,
 * heading-valid, calibration-valid, magnetic-used, magnetic-data-fresh.
 * Binary: original 64-byte packet, including timestamps/status/counters/CRC.
 * Change to IMU_TELEMETRY_FORMAT_BINARY to restore the original output.
 */
#define IMU_TELEMETRY_FORMAT_BINARY     0U
#define IMU_TELEMETRY_FORMAT_FIREWATER  1U
#ifndef IMU_TELEMETRY_OUTPUT_FORMAT
#define IMU_TELEMETRY_OUTPUT_FORMAT IMU_TELEMETRY_FORMAT_FIREWATER
#endif
#if (IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY) && \
    (IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_FIREWATER)
#error Unsupported IMU telemetry output format
#endif

/* Raw measurement fields are sensor-native; optional attitude is body-frame. */
#define IMU_TELEMETRY_FLAG_SAMPLE_VALID  (1U << 0)
#define IMU_TELEMETRY_FLAG_SENSOR_AXES   (1U << 1)
#define IMU_TELEMETRY_FLAG_STALE         (1U << 2)
#define IMU_TELEMETRY_FLAG_DRIVER_FAULT  (1U << 3)
#define IMU_TELEMETRY_FLAG_UNCALIBRATED  (1U << 4)

typedef struct {
    uint32_t timestamp_us;
    uint32_t sequence;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
    uint16_t flags;
    uint8_t driver_state;
    uint8_t driver_error;
    uint32_t sample_drops;
    uint32_t spi_errors;
    uint32_t timeouts;
    /* Extra text diagnostics; the original binary v1 layout stays unchanged. */
    uint8_t who_am_i;
    uint8_t bad_register;
    uint8_t readback_expected;
    uint8_t readback_actual;
    uint32_t drdy_count;
    uint32_t samples;
    /* Optional attitude extension. Binary v1 remains exactly 64 bytes.
     * FireWater extends the raw fields with roll,pitch,yaw (deg),valid: 11 total.
     * Startup/fault angles are zero placeholders with valid=0 plus #ATT diagnostics.
     */
    uint32_t attitude_present, attitude_flags, attitude_timestamp_us;
    uint32_t attitude_state, attitude_error, calibration_samples, calibration_restarts;
    int32_t attitude_mdeg[3];
    uint32_t mag_present, heading_valid, mag_calibrated, mag_used, mag_healthy;
    int32_t mag_raw_mut[3]; /* raw body field, milli-microtesla, for calibration */
} ImuTelemetryRecord;

typedef struct {
    uint32_t frames_started;
    uint32_t frames_completed;
    uint32_t frames_dropped;
    uint32_t uart_errors;
    uint32_t tx_timeouts;
    uint32_t start_errors;
    uint32_t aborts;
    uint32_t abort_timeouts;
    uint32_t last_hal_error;
    uint32_t status_frames;
    uint8_t busy;
    uint8_t faulted;
    uint32_t attitude_frames;
    uint32_t attitude_status_frames;
} ImuTelemetryStats;

/* Call once in main, after UART/DMA and the microsecond timer initialization.
 * UART TX belongs exclusively to this module; DMA is Normal, byte-wide.
 * UART and its DMA IRQ must use preemption priority 8 (group 4).
 * The static DMA buffer must be linked into DMA-accessible SRAM, not CCM.
 */
void ImuTelemetry_Init(UART_HandleTypeDef *uart, uint32_t now_us);

/* Main-loop only; latest must remain stable during this call. NULL skips the
 * scheduled opportunity. Binary mode can send status-only fault records.
 * FireWater sends only valid numeric samples. While invalid, send a separate
 * #IMU text diagnostic at most once a second (view in the raw serial console).
 * Always service frequently, including when there are no new IMU samples.
 */
void ImuTelemetry_Service(const ImuTelemetryRecord *latest, uint32_t now_us);
void ImuTelemetry_GetStats(ImuTelemetryStats *stats);

/* Forward the corresponding HAL UART callbacks from the application. */
void ImuTelemetry_OnTxComplete(UART_HandleTypeDef *uart);
void ImuTelemetry_OnError(UART_HandleTypeDef *uart);
void ImuTelemetry_OnAbortComplete(UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif
#endif
