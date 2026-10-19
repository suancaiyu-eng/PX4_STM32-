#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "mpu6000.h"
#include "lsm303d_mag.h"

#define ATTITUDE_PUBLICATION_US 5000U
#define ATTITUDE_CALIBRATION_SAMPLES 2000U
#define ATTITUDE_VALID          (1U << 0)
#define ATTITUDE_RELATIVE_YAW   (1U << 1)
#define ATTITUDE_GYRO_CALIBRATED (1U << 2)
#define ATTITUDE_ACCEL_USED     (1U << 3)
#define ATTITUDE_MAG_USED       (1U << 4)
#define ATTITUDE_MAG_CALIBRATED (1U << 5)
#define ATTITUDE_HEADING_VALID  (1U << 6)

typedef enum {
    ATTITUDE_WAITING = 0, ATTITUDE_CALIBRATING, ATTITUDE_RUNNING, ATTITUDE_FAULT
} AttitudeState;
typedef enum {
    ATTITUDE_ERROR_NONE = 0, ATTITUDE_ERROR_INPUT, ATTITUDE_ERROR_TIMING,
    ATTITUDE_ERROR_GAP, ATTITUDE_ERROR_SENSOR, ATTITUDE_ERROR_NUMERIC
} AttitudeError;

/* Body FRD: +X board arrow/forward, +Y right, +Z down. Hamilton quaternion
 * [w,x,y,z] rotates body vectors into a local NED frame with arbitrary heading.
 * Euler angles are radians: positive roll right-wing-down, pitch nose-up,
 * yaw clockwise viewed from above. With HEADING_VALID, yaw is magnetic heading
 * (declination=0); otherwise heading is unqualified/relative. Not true-north yaw.
 * All members are 32 bits for a predictable debugger/host snapshot layout.
 * Quaternion, Euler, rates and accel share timestamp_us at the 200 Hz publication.
 */
typedef struct {
    uint32_t timestamp_us, sequence, state, error, flags;
    float quaternion[4];
    float euler_rad[3];
    float gyro_bias_rad_s[3];
    float body_rates_rad_s[3];
    float accel_body_m_s2[3];
    uint32_t input_samples, prediction_updates, correction_updates, publications;
    uint32_t calibration_samples, calibration_restarts, accel_rejections;
    uint32_t timing_faults, gap_faults;
    /* Heading state: 0=none, 1=uncalibrated aiding, 2=calibrated aiding,
     * 3=disturbed/rejected, 4=stale, 5=qualifying/reacquiring. */
    uint32_t heading_state, mag_samples, mag_corrections, mag_rejections;
    uint32_t mag_stale_events, mag_timestamp_us, mag_sequence;
    float magnetic_body_uT[3], magnetic_norm_uT, magnetic_innovation_rad;
    float magnetic_bias_rad_s[3];
} AttitudeEstimate;

/* Foreground only. Startup calibration requires a continuous quiet window.
 * A fault latches until explicit Init; never restart an estimator in flight.
 * Every timestamped sample must be consumed, even though Euler publishes at 200 Hz.
 */
void AttitudeEstimator_Init(void);
void AttitudeEstimator_Consume(const MPU6000_Sample *sample,
                               const MPU6000_Measurement *measurement);
void AttitudeEstimator_Service(uint32_t now_us, bool sensor_healthy);
void AttitudeEstimator_GetEstimate(AttitudeEstimate *estimate);
void AttitudeEstimator_SetMagneticSample(const MagSample *sample);
void AttitudeEstimator_MapToBody(const float sensor[3], float body[3]);
#endif
