#ifndef MPU6000_H
#define MPU6000_H

#include <stdbool.h>
#include <stdint.h>

/* Sensor-frame measurements. No assumed board rotation or automatic bias removal. */
#define MPU6000_QUEUE_CAPACITY 16U
#define MPU6000_BURST_SIZE 16U /* command + INT_STATUS + accel/temp/gyro */
#define MPU6000_SAMPLE_FIRST 0x0001U
#define MPU6000_SAMPLE_GAP   0x0002U

typedef enum {
    MPU6000_OFF = 0, MPU6000_POWER_WAIT, MPU6000_RESET_WAIT,
    MPU6000_SIGNAL_WAIT, MPU6000_WAKE_WAIT, MPU6000_CONFIGURE,
    MPU6000_RUNNING, MPU6000_FAULT
} MPU6000_State;

typedef enum {
    MPU6000_ERROR_NONE = 0, MPU6000_ERROR_ID, MPU6000_ERROR_REGISTER_IO,
    MPU6000_ERROR_READBACK, MPU6000_ERROR_DMA_START, MPU6000_ERROR_SPI,
    MPU6000_ERROR_DMA_TIMEOUT, MPU6000_ERROR_DRDY_TIMEOUT,
    MPU6000_ERROR_DATA_STATUS
} MPU6000_Error;

typedef struct {
    uint32_t timestamp_us; /* DRDY interrupt timestamp, NOT transfer-complete time */
    uint32_t sequence;
    uint32_t dt_us;
    int16_t accel_raw[3];
    int16_t temperature_raw;
    int16_t gyro_raw[3];
    uint16_t flags;
} MPU6000_Sample;

typedef struct {
    float accel_m_s2[3];
    float gyro_rad_s[3];
    float temperature_c;
} MPU6000_Measurement;

typedef struct {
    MPU6000_State state;
    MPU6000_Error last_error;
    uint8_t who_am_i;
    uint8_t bad_register;
    uint8_t readback_expected;
    uint8_t readback_actual;
    uint32_t drdy_count;
    uint32_t samples;
    uint32_t busy_drops;
    uint32_t queue_drops;
    uint32_t invalid_samples;
    uint32_t spi_errors;
    uint32_t timeouts;
    uint32_t last_sample_us;
    uint32_t max_transfer_us;
} MPU6000_Status;

/* Init once after GPIO/SPI/DMA/TIM2 initialization. TIM2 must already be running. */
void MPU6000_Init(uint32_t now_us);
/* Foreground staged initialization uses bounded register IO. The running path
 * never waits and never automatically restarts a faulted sensor. */
void MPU6000_Service(uint32_t now_us);
bool MPU6000_PopSample(MPU6000_Sample *sample);
void MPU6000_Convert(const MPU6000_Sample *sample, MPU6000_Measurement *measurement);
void MPU6000_GetStatus(MPU6000_Status *status);
bool MPU6000_IsHealthy(uint32_t now_us);
/* Explicit recovery only; caller must ensure the vehicle is disarmed. */
bool MPU6000_RestartDisarmed(uint32_t now_us);

/* Called only from the board's EXTI / SPI callbacks (same NVIC priority). */
void MPU6000_OnDataReady(uint32_t now_us);
void MPU6000_OnDmaComplete(uint32_t now_us);
void MPU6000_OnSpiError(void);

#endif
