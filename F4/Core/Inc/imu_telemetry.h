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
#define IMU_TELEMETRY_TX_BUFFER_SIZE  320U
#define IMU_TELEMETRY_STATUS_PERIOD_US 1000000U

/* Legacy FireWater console: 1 selects RC-only and suppresses boot diagnostics.
 * SeekFree selects its source with IMU_TELEMETRY_VIEW below; keep this at 1
 * for a clean numeric console. Set 0 only when restoring legacy diagnostics. */
#ifndef IMU_TELEMETRY_RC_ONLY
#define IMU_TELEMETRY_RC_ONLY 1U
#endif

/* Select ONE wire format, then rebuild. All use the same DMA owner.
 * SeekFree (default): one selected numeric data set, at most eight channels.
 * FireWater raw-only: mpu6000:ax,ay,az,gx,gy,gz,temp\n (int16 raw counts).
 * The flight application defaults to 18 channels: raw IMU, attitude, raw field,
 * heading-valid, calibration-valid, magnetic-used, magnetic-data-fresh.
 * Binary: original 64-byte packet, including timestamps/status/counters/CRC.
 * Change to FIREWATER / BINARY to restore the corresponding legacy encoder.
 */
#define IMU_TELEMETRY_FORMAT_BINARY     0U
#define IMU_TELEMETRY_FORMAT_FIREWATER  1U
#define IMU_TELEMETRY_FORMAT_SEEKFREE   2U
#ifndef IMU_TELEMETRY_OUTPUT_FORMAT
#define IMU_TELEMETRY_OUTPUT_FORMAT IMU_TELEMETRY_FORMAT_SEEKFREE
#endif
#if (IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_BINARY) && \
    (IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_FIREWATER) && \
    (IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_SEEKFREE)
#error Unsupported IMU telemetry output format
#endif

/* SeekFree Printf presets. Keep ONE view definition enabled, then rebuild.
 * No floating printf; all decimal values are formatted from integer units. */
#define IMU_TELEMETRY_VIEW_GYRO      0U /* gyro:x,y,z\n, sensor axes, deg/s */
#define IMU_TELEMETRY_VIEW_ACCEL     1U /* accel:x,y,z\n, sensor axes, g */
#define IMU_TELEMETRY_VIEW_ATTITUDE  2U /* attitude:roll,pitch,yaw,heading_valid\n, deg */
#define IMU_TELEMETRY_VIEW_MAG       3U /* mag:x,y,z,calibrated\n, body axes, uT */
#define IMU_TELEMETRY_VIEW_BARO      4U /* baro:Pa,centiC,relative_mm,filtered_mm,flags,model,state,error\n */
#define IMU_TELEMETRY_VIEW_RC        5U /* rc:CH1..CH8\n, see board_link_telem.h */
#ifndef IMU_TELEMETRY_VIEW
#if defined(BARO_TELEMETRY_ENABLED) && BARO_TELEMETRY_ENABLED
#define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_BARO /* dedicated Keil barometer target */
#else
#define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_GYRO
/* #define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_ACCEL */
/* #define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_ATTITUDE */
/* #define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_MAG */
/* #define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_BARO */
/* #define IMU_TELEMETRY_VIEW IMU_TELEMETRY_VIEW_RC */
#endif
#endif
#if IMU_TELEMETRY_VIEW > IMU_TELEMETRY_VIEW_RC
#error Unsupported SeekFree telemetry view
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
    int32_t baro_pressure_pa, baro_temperature_centi_c;
    int32_t baro_relative_mm, baro_filtered_mm;
    uint32_t baro_flags, baro_model, baro_state, baro_error;
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
    uint32_t text_queued;
    uint32_t text_started;
    uint32_t text_dropped;
} ImuTelemetryStats;

/* Call once in main, after UART/DMA and the microsecond timer initialization.
 * UART TX belongs exclusively to this module; DMA is Normal, byte-wide.
 * UART and its DMA IRQ must use preemption priority 8 (group 4).
 * The static DMA buffer must be linked into DMA-accessible SRAM, not CCM.
 */
void ImuTelemetry_Init(UART_HandleTypeDef *uart, uint32_t now_us);

/* Main-loop only; latest must remain stable during this call. NULL skips the
 * scheduled opportunity. Binary mode can send status-only fault records.
 * SeekFree gyro/accel/attitude/mag skip invalid samples without diagnostic text.
 * Barometer includes explicit flags/state/error. RC uses the text queue.
 * Legacy FireWater sends only valid numeric samples. While invalid, send a separate
 * #IMU text diagnostic at most once a second (view in the raw serial console).
 * Always service frequently, including when there are no new IMU samples.
 */
void ImuTelemetry_Service(const ImuTelemetryRecord *latest, uint32_t now_us);
void ImuTelemetry_GetStats(ImuTelemetryStats *stats);
/* Main-loop only. Copies a complete LF-terminated text line into a four-entry
 * queue. Returns 1 on success, 0 if invalid/full or binary format is selected.
 * The existing DMA owner schedules these lines between regular IMU records.
 */
uint8_t ImuTelemetry_QueueText(const char *text, uint16_t length);

/* Forward the corresponding HAL UART callbacks from the application. */
void ImuTelemetry_OnTxComplete(UART_HandleTypeDef *uart);
void ImuTelemetry_OnError(UART_HandleTypeDef *uart);
void ImuTelemetry_OnAbortComplete(UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif
#endif
