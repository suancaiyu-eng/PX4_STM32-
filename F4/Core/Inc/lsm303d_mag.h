#ifndef LSM303D_MAG_H
#define LSM303D_MAG_H
#include <stdbool.h>
#include <stdint.h>

#define MAG_SAMPLE_VALID 1U
#define MAG_SAMPLE_CALIBRATED 2U
typedef struct {
    uint32_t timestamp_us, sequence, flags;
    float raw_body_uT[3];
    float field_body_uT[3];
    float reference_uT;
} MagSample;
typedef struct {
    uint32_t state, error, who_am_i, bad_register, expected, actual;
    uint32_t samples, not_ready, overruns, bus_errors, invalid_samples;
    uint32_t last_sample_us, max_interval_us, calibration_valid;
    int32_t raw[3];
    float field_uT[3], norm_uT;
    uint32_t sample_valid, calibration_error;
} MagStatus;

/* State 0=waiting for IMU, 1=configuring, 2=running, 3=latched fault.
 * Errors 1=ID, 2=bus transfer, 3=readback, 4=sample timeout.
 * Only LSM303D (WHO=0x49) on FMUv2 PC15 is supported, not external HMC5883.
 */
void Mag_Init(uint32_t now_us);
void Mag_Service(uint32_t now_us, bool imu_running);
bool Mag_PopSample(MagSample *sample);
void Mag_GetStatus(MagStatus *status);
/* Port contract: positive=success, zero=bus busy/retry, negative=error. */
int Mag_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length);
#endif
