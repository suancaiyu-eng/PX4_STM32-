#include "attitude_estimator.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define GRAVITY 9.80665f
#define KP 2.0f
#define ACCEL_TAU_S 0.007957747f /* first-order 20 Hz at the full IMU rate */
#define STALE_US 5000U
#define CALIBRATION_MIN_US 1900000U

/* Mahony gravity-vector feedback, quaternion kinematics and normalization.
 * Adapted conceptually from the reference Modules/attitude_estimator_mahony.c;
 * implemented for timestamped samples and an explicit lifecycle. Independent
 * tilt-compensated magnetic heading feedback constrains yaw without injecting
 * magnetic disturbance into roll/pitch. Magnetic calibration is never assumed.
 */
static AttitudeEstimate output;
static float q[4], accel_filtered[3], correction[3];
static float mean_gyro[3], m2_gyro[3], mean_accel[3], m2_accel[3];
static uint32_t previous_us, previous_sequence, correction_us, publication_us;
static uint32_t calibration_started_us;
static bool have_previous;
static MagSample magnetic;
static float magnetic_correction[3], startup_bias[3], reference_norm, reference_dip;
static uint32_t magnetic_token, processed_magnetic_token, magnetic_good;
static uint32_t previous_magnetic_us;
static bool have_magnetic, heading_aligned, mag_was_fresh;

static bool finite_float(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static float norm_squared(const float v[3])
{
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

static void fail(AttitudeError error)
{
    if (output.state == ATTITUDE_FAULT) { return; }
    output.state = ATTITUDE_FAULT;
    output.error = (uint32_t)error;
    output.flags &= ~ATTITUDE_VALID;
    output.flags &= ~(ATTITUDE_MAG_USED | ATTITUDE_HEADING_VALID);
}

void AttitudeEstimator_MapToBody(const float sensor[3], float body[3])
{
    float x = sensor[0], y = sensor[1], z = sensor[2];
    /* FMUv2 raw MPU6000: [Y,-X,Z]. This composes PX4's MPU Y/Z flip
     * and FMUv2 ROTATION_ROLL_180_YAW_270 (-R 14). Not a mounting trim. */
    body[0] = y; body[1] = -x; body[2] = z;
}

static void reset_calibration(void)
{
    if (output.calibration_samples != 0U) { ++output.calibration_restarts; }
    output.calibration_samples = 0U;
    memset(mean_gyro, 0, sizeof(mean_gyro));
    memset(m2_gyro, 0, sizeof(m2_gyro));
    memset(mean_accel, 0, sizeof(mean_accel));
    memset(m2_accel, 0, sizeof(m2_accel));
}

void AttitudeEstimator_Init(void)
{
    memset(&output, 0, sizeof(output));
    memset(q, 0, sizeof(q));
    memset(accel_filtered, 0, sizeof(accel_filtered));
    memset(correction, 0, sizeof(correction));
    q[0] = output.quaternion[0] = 1.0f;
    output.flags = ATTITUDE_RELATIVE_YAW;
    have_previous = false;
    previous_us = previous_sequence = correction_us = publication_us = 0U;
    calibration_started_us = 0U;
    reset_calibration();
    memset(&magnetic,0,sizeof(magnetic));
    memset(magnetic_correction,0,sizeof(magnetic_correction));
    memset(startup_bias,0,sizeof(startup_bias));
    reference_norm=reference_dip=0.0f;
    magnetic_token=processed_magnetic_token=magnetic_good=0U;
    previous_magnetic_us=0U;
    have_magnetic=heading_aligned=mag_was_fresh=false;
}

void AttitudeEstimator_SetMagneticSample(const MagSample *sample)
{
    if(sample == NULL) { have_magnetic=false; return; }
    magnetic=*sample; have_magnetic=true; ++magnetic_token;
}

static void magnetic_unavailable(uint32_t state)
{
    memset(magnetic_correction,0,sizeof(magnetic_correction));
    output.flags &= ~(ATTITUDE_MAG_USED | ATTITUDE_HEADING_VALID);
    output.flags |= ATTITUDE_RELATIVE_YAW;
    output.heading_state=state;
    magnetic_good=0U;
}

static bool magnetic_fresh(uint32_t now_us)
{
    return have_magnetic && (magnetic.flags & MAG_SAMPLE_VALID) != 0U &&
           (uint32_t)(now_us-magnetic.timestamp_us) <= 50000U;
}

static void update_magnetic(uint32_t now_us, const float gyro[3])
{
    float m[3], north, east, down, horizontal, norm, innovation, gravity[3], mag_dt;
    uint32_t i;
    if(!magnetic_fresh(now_us)) {
        if(mag_was_fresh) { ++output.mag_stale_events; }
        mag_was_fresh=false;
        magnetic_unavailable(have_magnetic ? 4U : 0U);
        return;
    }
    mag_was_fresh=true;
    /* A body vector must not be reinterpreted as a new observation twice.
     * The ~100Hz magnetic correction is held between new observations. */
    if(processed_magnetic_token == magnetic_token) { return; }
    processed_magnetic_token=magnetic_token;
    mag_dt=(float)(uint32_t)(magnetic.timestamp_us-previous_magnetic_us)*0.000001f;
    if(mag_dt>0.05f) { mag_dt=0.0f; }
    previous_magnetic_us=magnetic.timestamp_us;
    ++output.mag_samples;
    output.mag_timestamp_us=magnetic.timestamp_us;
    output.mag_sequence=magnetic.sequence;
    norm=0.0f;
    for(i=0U;i<3U;++i) {
        m[i]=magnetic.field_body_uT[i];
        if(!finite_float(m[i]) || fabsf(m[i]) > 250.0f) {
            ++output.mag_rejections; magnetic_unavailable(3U); return;
        }
        output.magnetic_body_uT[i]=m[i]; norm+=m[i]*m[i];
    }
    norm=sqrtf(norm); output.magnetic_norm_uT=norm;
    if(norm < 10.0f || norm > 100.0f ||
       ((magnetic.flags & MAG_SAMPLE_CALIBRATED) != 0U &&
        (!finite_float(magnetic.reference_uT) || magnetic.reference_uT < 15.0f ||
         magnetic.reference_uT > 80.0f || norm < 0.75f*magnetic.reference_uT ||
         norm > 1.25f*magnetic.reference_uT))) {
        ++output.mag_rejections; magnetic_unavailable(3U); return;
    }
    north=(1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]))*m[0] +
          2.0f*(q[1]*q[2]-q[0]*q[3])*m[1] + 2.0f*(q[1]*q[3]+q[0]*q[2])*m[2];
    east=2.0f*(q[1]*q[2]+q[0]*q[3])*m[0] +
         (1.0f-2.0f*(q[1]*q[1]+q[3]*q[3]))*m[1] + 2.0f*(q[2]*q[3]-q[0]*q[1])*m[2];
    gravity[0]=2.0f*(q[1]*q[3]-q[0]*q[2]);
    gravity[1]=2.0f*(q[0]*q[1]+q[2]*q[3]);
    gravity[2]=1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]);
    down=gravity[0]*m[0]+gravity[1]*m[1]+gravity[2]*m[2];
    horizontal=sqrtf(north*north+east*east);
    innovation=atan2f(-east,north);
    output.magnetic_innovation_rad=innovation;
    if(horizontal < 5.0f || (heading_aligned &&
       (fabsf(innovation)>0.43633231f || fabsf(down/norm-reference_dip)>0.20f ||
        norm<0.75f*reference_norm || norm>1.25f*reference_norm))) {
        ++output.mag_rejections; magnetic_unavailable(3U); return;
    }
    if(!heading_aligned) {
        /* Require 20 fresh observations, not 20 main-loop iterations. Initial
         * alignment is only permitted with low angular rate after gyro startup. */
        if(norm_squared(gyro)>0.08726646f*0.08726646f) { magnetic_unavailable(5U); return; }
        if(magnetic_good == 0U) { reference_norm=norm; reference_dip=down/norm; }
        if(fabsf(norm/reference_norm-1.0f)>0.15f || fabsf(down/norm-reference_dip)>0.1f) {
            magnetic_unavailable(5U); return;
        }
        ++magnetic_good;
        output.heading_state=5U;
        if(magnetic_good < 20U) { return; }
        {
            float c=cosf(0.5f*innovation), s=sinf(0.5f*innovation), old[4];
            memcpy(old,q,sizeof(q));
            q[0]=c*old[0]-s*old[3]; q[1]=c*old[1]-s*old[2];
            q[2]=c*old[2]+s*old[1]; q[3]=c*old[3]+s*old[0];
        }
        heading_aligned=true; innovation=0.0f;
    } else if(magnetic_good < 20U) {
        ++magnetic_good;
        if(magnetic_good < 20U) {
            memset(magnetic_correction,0,sizeof(magnetic_correction));
            output.flags &= ~(ATTITUDE_MAG_USED | ATTITUDE_HEADING_VALID);
            output.flags |= ATTITUDE_RELATIVE_YAW;
            output.heading_state=5U; return;
        }
    }
    for(i=0U;i<3U;++i) { magnetic_correction[i]=1.0f*innovation*gravity[i]; }
    output.flags |= ATTITUDE_MAG_USED;
    if((magnetic.flags & MAG_SAMPLE_CALIBRATED) != 0U) {
        output.flags |= ATTITUDE_MAG_CALIBRATED | ATTITUDE_HEADING_VALID;
        output.flags &= ~ATTITUDE_RELATIVE_YAW;
        output.heading_state=2U;
        /* Slow, bounded online gyro bias estimation, along the observable
         * heading direction only. Freeze on rejected/stale/uncalibrated data
         * and during fast turns where sample latency could masquerade as bias. */
        if(norm_squared(gyro) < 0.12186934f) {
            for(i=0U;i<3U;++i) {
                float next=output.magnetic_bias_rad_s[i]-0.02f*innovation*gravity[i]*mag_dt;
                if(next>0.01745329f) { next=0.01745329f; }
                if(next< -0.01745329f) { next= -0.01745329f; }
                output.magnetic_bias_rad_s[i]=next;
                output.gyro_bias_rad_s[i]=startup_bias[i]+next;
            }
        }
    } else {
        output.flags &= ~(ATTITUDE_MAG_CALIBRATED | ATTITUDE_HEADING_VALID);
        output.flags |= ATTITUDE_RELATIVE_YAW;
        output.heading_state=1U;
    }
    ++output.mag_corrections;
}

static void publish(const MPU6000_Sample *sample, const float gyro[3], const float accel[3])
{
    uint32_t i;
    float sin_pitch = 2.0f * (q[0]*q[2] - q[3]*q[1]);
    /* Roundoff near vertical flight must not feed asinf outside [-1,1]. */
    if (sin_pitch > 1.0f) { sin_pitch = 1.0f; }
    if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }
    output.euler_rad[0] = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]),
                                1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
    output.euler_rad[1] = asinf(sin_pitch);
    output.euler_rad[2] = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]),
                                1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
    for (i = 0U; i < 4U; ++i) { output.quaternion[i] = q[i]; }
    for (i = 0U; i < 3U; ++i) {
        output.body_rates_rad_s[i] = gyro[i];
        output.accel_body_m_s2[i] = accel[i];
    }
    output.timestamp_us = sample->timestamp_us;
    output.sequence = sample->sequence;
    output.flags |= ATTITUDE_VALID;
    ++output.publications;
}

static void calibrate(const MPU6000_Sample *sample, const float gyro[3], const float accel[3])
{
    uint32_t i;
    float accel_norm = norm_squared(accel);
    float count, roll, pitch, cr, sr, cp, sp;
    /* Quiet-window checks reject motion and invalid gravity before averaging.
     * A slow constant turn cannot be distinguished perfectly from gyro bias
     * using six axes; startup is deliberately documented as disarmed/stationary.
     */
    if (accel_norm < 0.9f*0.9f*GRAVITY*GRAVITY ||
        accel_norm > 1.1f*1.1f*GRAVITY*GRAVITY ||
        norm_squared(gyro) > 0.08726646f*0.08726646f) {
        reset_calibration();
        return;
    }
    if (output.calibration_samples == 0U) { calibration_started_us = sample->timestamp_us; }
    ++output.calibration_samples;
    count = (float)output.calibration_samples;
    for (i = 0U; i < 3U; ++i) {
        float delta = gyro[i] - mean_gyro[i];
        mean_gyro[i] += delta / count;
        m2_gyro[i] += delta * (gyro[i] - mean_gyro[i]);
        delta = accel[i] - mean_accel[i];
        mean_accel[i] += delta / count;
        m2_accel[i] += delta * (accel[i] - mean_accel[i]);
    }
    if (output.calibration_samples < ATTITUDE_CALIBRATION_SAMPLES ||
        (uint32_t)(sample->timestamp_us - calibration_started_us) < CALIBRATION_MIN_US) { return; }
    for (i = 0U; i < 3U; ++i) {
        if (m2_gyro[i] / count > 0.00349066f*0.00349066f ||
            m2_accel[i] / count > 0.15f*0.15f) {
            reset_calibration();
            return;
        }
    }
    /* Specific force at rest is opposite gravity. Arbitrary initial tilt is
     * supported; yaw starts at zero. No one-position accelerometer calibration. */
    roll = atan2f(-mean_accel[1], -mean_accel[2]);
    pitch = atan2f(mean_accel[0], sqrtf(mean_accel[1]*mean_accel[1] + mean_accel[2]*mean_accel[2]));
    cr = cosf(0.5f*roll); sr = sinf(0.5f*roll);
    cp = cosf(0.5f*pitch); sp = sinf(0.5f*pitch);
    q[0] = cr*cp; q[1] = sr*cp; q[2] = cr*sp; q[3] = -sr*sp;
    for (i = 0U; i < 3U; ++i) {
        output.gyro_bias_rad_s[i] = mean_gyro[i];
        startup_bias[i] = mean_gyro[i];
        accel_filtered[i] = accel[i];
    }
    output.state = ATTITUDE_RUNNING;
    output.flags |= ATTITUDE_GYRO_CALIBRATED;
    correction_us = publication_us = sample->timestamp_us;
    {
        float corrected[3];
        for (i = 0U; i < 3U; ++i) { corrected[i] = gyro[i] - mean_gyro[i]; }
        publish(sample, corrected, accel);
    }
}

void AttitudeEstimator_Consume(const MPU6000_Sample *sample, const MPU6000_Measurement *measurement)
{
    float gyro[3], accel[3], omega[3], before[4], dt, alpha, reciprocal;
    uint32_t i, elapsed = 0U;
    if (output.state == ATTITUDE_FAULT) { return; }
    if (sample == NULL || measurement == NULL) { fail(ATTITUDE_ERROR_INPUT); return; }
    for (i = 0U; i < 3U; ++i) {
        if (!finite_float(measurement->gyro_rad_s[i]) ||
            !finite_float(measurement->accel_m_s2[i]) ||
            fabsf(measurement->gyro_rad_s[i]) > 35.0f ||
            fabsf(measurement->accel_m_s2[i]) > 200.0f ||
            sample->gyro_raw[i] <= -32760 || sample->gyro_raw[i] >= 32760 ||
            sample->accel_raw[i] <= -32760 || sample->accel_raw[i] >= 32760) {
            fail(ATTITUDE_ERROR_INPUT); return;
        }
    }
    if (have_previous) {
        elapsed = (uint32_t)(sample->timestamp_us - previous_us);
        if ((uint32_t)(sample->sequence - previous_sequence) != 1U ||
            (sample->flags & (MPU6000_SAMPLE_FIRST | MPU6000_SAMPLE_GAP)) != 0U) {
            ++output.gap_faults; fail(ATTITUDE_ERROR_GAP); return;
        }
        if (elapsed < 500U || elapsed > 1500U || elapsed != sample->dt_us) {
            ++output.timing_faults; fail(ATTITUDE_ERROR_TIMING); return;
        }
    }
    previous_us = sample->timestamp_us;
    previous_sequence = sample->sequence;
    have_previous = true;
    ++output.input_samples;
    AttitudeEstimator_MapToBody(measurement->gyro_rad_s, gyro);
    AttitudeEstimator_MapToBody(measurement->accel_m_s2, accel);
    if (output.state != ATTITUDE_RUNNING) {
        output.state = ATTITUDE_CALIBRATING;
        calibrate(sample, gyro, accel);
        return;
    }
    dt = (float)elapsed * 0.000001f;
    alpha = dt / (ACCEL_TAU_S + dt);
    for (i = 0U; i < 3U; ++i) {
        gyro[i] -= output.gyro_bias_rad_s[i];
        accel_filtered[i] += alpha * (accel[i] - accel_filtered[i]);
    }
    if ((uint32_t)(sample->timestamp_us - correction_us) >= ATTITUDE_PUBLICATION_US) {
        float raw_norm = norm_squared(accel), filtered_norm = norm_squared(accel_filtered);
        memset(correction, 0, sizeof(correction));
        output.flags &= ~ATTITUDE_ACCEL_USED;
        /* Advance the phase, rather than scheduling from this sample. The real
         * MPU clock is slightly faster than 1 kHz; resetting to 'now' would
         * repeatedly wait six samples and silently reduce output to ~167 Hz. */
        correction_us += ((uint32_t)(sample->timestamp_us - correction_us) /
                          ATTITUDE_PUBLICATION_US) * ATTITUDE_PUBLICATION_US;
        ++output.correction_updates;
        update_magnetic(sample->timestamp_us,gyro);
        if (raw_norm > 0.85f*0.85f*GRAVITY*GRAVITY && raw_norm < 1.15f*1.15f*GRAVITY*GRAVITY &&
            filtered_norm > 0.85f*0.85f*GRAVITY*GRAVITY && filtered_norm < 1.15f*1.15f*GRAVITY*GRAVITY) {
            float measured[3], predicted[3];
            reciprocal = 1.0f / sqrtf(filtered_norm);
            for (i = 0U; i < 3U; ++i) { measured[i] = -accel_filtered[i]*reciprocal; }
            predicted[0] = 2.0f*(q[1]*q[3] - q[0]*q[2]);
            predicted[1] = 2.0f*(q[0]*q[1] + q[2]*q[3]);
            predicted[2] = 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]);
            correction[0] = KP*(measured[1]*predicted[2] - measured[2]*predicted[1]);
            correction[1] = KP*(measured[2]*predicted[0] - measured[0]*predicted[2]);
            correction[2] = KP*(measured[0]*predicted[1] - measured[1]*predicted[0]);
            output.flags |= ATTITUDE_ACCEL_USED;
        } else { ++output.accel_rejections; }
    }
    /* Immediately suppress stale gravity feedback during a strong acceleration,
     * including the four intermediate prediction-only samples. */
    reciprocal = norm_squared(accel);
    if (reciprocal < 0.85f*0.85f*GRAVITY*GRAVITY || reciprocal > 1.15f*1.15f*GRAVITY*GRAVITY) {
        memset(correction, 0, sizeof(correction));
        output.flags &= ~ATTITUDE_ACCEL_USED;
    }
    if(!magnetic_fresh(sample->timestamp_us)) { magnetic_unavailable(have_magnetic ? 4U : 0U); }
    for (i = 0U; i < 3U; ++i) { omega[i] = (gyro[i] + correction[i]+magnetic_correction[i])*0.5f*dt; }
    memcpy(before, q, sizeof(q));
    q[0] += -before[1]*omega[0] - before[2]*omega[1] - before[3]*omega[2];
    q[1] +=  before[0]*omega[0] + before[2]*omega[2] - before[3]*omega[1];
    q[2] +=  before[0]*omega[1] - before[1]*omega[2] + before[3]*omega[0];
    q[3] +=  before[0]*omega[2] + before[1]*omega[1] - before[2]*omega[0];
    reciprocal = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (!finite_float(reciprocal) || reciprocal < 0.25f || reciprocal > 4.0f) {
        fail(ATTITUDE_ERROR_NUMERIC); return;
    }
    reciprocal = 1.0f / sqrtf(reciprocal);
    for (i = 0U; i < 4U; ++i) { q[i] *= reciprocal; }
    ++output.prediction_updates;
    if ((uint32_t)(sample->timestamp_us - publication_us) >= ATTITUDE_PUBLICATION_US) {
        publication_us += ((uint32_t)(sample->timestamp_us - publication_us) /
                           ATTITUDE_PUBLICATION_US) * ATTITUDE_PUBLICATION_US;
        publish(sample, gyro, accel);
    }
}

void AttitudeEstimator_Service(uint32_t now_us, bool sensor_healthy)
{
    if (!have_previous || output.state == ATTITUDE_FAULT) { return; }
    if (!sensor_healthy) { fail(ATTITUDE_ERROR_SENSOR); }
    else if ((uint32_t)(now_us - previous_us) > STALE_US) {
        ++output.timing_faults; fail(ATTITUDE_ERROR_TIMING);
    }
    if(!magnetic_fresh(now_us)) {
        if(mag_was_fresh) { ++output.mag_stale_events; mag_was_fresh=false; }
        magnetic_unavailable(have_magnetic ? 4U : 0U);
    }
}

void AttitudeEstimator_GetEstimate(AttitudeEstimate *estimate)
{
    if (estimate != NULL) { *estimate = output; }
}
