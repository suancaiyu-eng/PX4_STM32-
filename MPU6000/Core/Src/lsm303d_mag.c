#include "lsm303d_mag.h"
#include "mag_calibration_params.h"
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <string.h>

static MagStatus status;
static MagSample latest;
static bool pending;
static uint32_t step, poll_phase, started_us;
static uint8_t temporary;
static uint32_t check_phase, check_index;
static const float bias[3] = MAG_CALIBRATION_BIAS;
static const float matrix[3][3] = MAG_CALIBRATION_MATRIX;
/* Accel power-down with BDU; high-resolution 100Hz mag; +/-2gauss; continuous.
 * MAG 100Hz is allowed with accelerometer power-down (ST Table 47). */
static const uint8_t registers[] = {0x20U,0x24U,0x25U,0x26U};
static const uint8_t values[] = {0x0FU,0x74U,0x00U,0x00U};

static bool calibration_valid(void)
{
    uint32_t crc=0xffffffffU, i,j,k,bits;
    float all[13], determinant;
    if(!MAG_CALIBRATION_VALID) { return false; }
    for(i=0U;i<3U;++i) { all[i]=bias[i]; }
    for(i=0U;i<3U;++i) for(j=0U;j<3U;++j) { all[3U+i*3U+j]=matrix[i][j]; }
    all[12]=MAG_CALIBRATION_REFERENCE_UT;
    for(i=0U;i<13U;++i) {
        if(!(all[i]>=-FLT_MAX && all[i]<=FLT_MAX)) { return false; }
        memcpy(&bits,&all[i],sizeof(bits));
        for(j=0U;j<4U;++j) {
            crc^=(bits>>(j*8U))&0xffU;
            for(k=0U;k<8U;++k) { crc=(crc>>1U)^((crc&1U)?0xedb88320U:0U); }
        }
    }
    if((crc^0xffffffffU)!=MAG_CALIBRATION_CRC32) { return false; }
    for(i=0U;i<3U;++i) {
        if(fabsf(bias[i])>200.0f || matrix[i][i]<0.1f || matrix[i][i]>5.0f) { return false; }
        for(j=0U;j<3U;++j) {
            if(fabsf(matrix[i][j])>5.0f || fabsf(matrix[i][j]-matrix[j][i])>0.0001f) { return false; }
        }
    }
    determinant=matrix[0][0]*(matrix[1][1]*matrix[2][2]-matrix[1][2]*matrix[2][1])-
                matrix[0][1]*(matrix[1][0]*matrix[2][2]-matrix[1][2]*matrix[2][0])+
                matrix[0][2]*(matrix[1][0]*matrix[2][1]-matrix[1][1]*matrix[2][0]);
    return matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0]>0.0001f &&
           determinant>0.0001f && all[12]>=15.0f && all[12]<=80.0f;
}

static void fault(uint32_t error, uint8_t reg)
{
    status.state = 3U; status.error = error; status.bad_register = reg;
    latest.flags = 0U; pending = false;
}

static int transfer(uint8_t reg, bool read, uint8_t value, uint8_t *result)
{
    uint8_t tx[2] = {(uint8_t)(reg | (read ? 0x80U : 0U)), value};
    uint8_t rx[2] = {0U,0U};
    int ok = Mag_Port_Transfer(tx, rx, 2U);
    if (ok < 0) { ++status.bus_errors; fault(2U, reg); }
    if (ok > 0 && result != NULL) { *result = rx[1]; }
    return ok;
}

void Mag_Init(uint32_t now_us)
{
    memset(&status,0,sizeof(status)); memset(&latest,0,sizeof(latest));
    status.calibration_valid = calibration_valid() ? 1U : 0U;
    status.calibration_error = MAG_CALIBRATION_VALID && !status.calibration_valid ? 1U : 0U;
    pending = false; step = 0U; temporary = 0U;
    poll_phase = started_us = now_us;
    check_phase=now_us; check_index=0U;
}

static void configure(uint32_t now_us)
{
    uint8_t actual = 0U, reg, value;
    int ok;
    if (step == 0U) {
        ok = transfer(0x0FU,true,0U,&actual);
        if (ok <= 0) { return; }
        status.who_am_i = actual;
        if (actual != 0x49U) { fault(1U,0x0FU); return; }
    } else if (step <= 8U) {
        /* ST silicon workaround also used by PX4's LSM303D::disable_i2c:
         * preserve other bits while preventing shared SPI traffic being taken
         * as I2C. Only execute after exact LSM303D identity validation. */
        if (step == 1U || step == 3U || step == 5U || step == 7U) {
            reg = step == 5U ? 0x15U : 0x02U;
            if (transfer(reg,true,0U,&temporary) <= 0) { return; }
        } else {
            reg = step == 6U ? 0x15U : 0x02U;
            value = step == 2U ? (uint8_t)(temporary | 0x10U) :
                    step == 4U ? (uint8_t)(temporary & 0xF7U) :
                    step == 6U ? (uint8_t)(temporary | 0x80U) : (uint8_t)(temporary & 0xE7U);
            if (transfer(reg,false,value,NULL) <= 0) { return; }
        }
    } else {
        uint32_t item = (step-9U)/2U;
        reg = registers[item]; value = values[item];
        if (((step-9U) & 1U) == 0U) {
            if (transfer(reg,false,value,NULL) <= 0) { return; }
        } else {
            if (transfer(reg,true,0U,&actual) <= 0) { return; }
            if (actual != value) {
                status.expected=value; status.actual=actual; fault(3U,reg); return;
            }
        }
    }
    ++step;
    if (step == 17U) {
        status.state=2U; poll_phase=started_us=check_phase=now_us;
    }
}

void Mag_Service(uint32_t now_us, bool imu_running)
{
    uint8_t tx[8] = {0xC7U,0U,0U,0U,0U,0U,0U,0U};
    uint8_t rx[8] = {0U};
    uint32_t i,j,interval;
    bool saturated=false;
    float centered[3];
    int ok;
    if (!imu_running || status.state == 3U) { return; }
    if (status.state < 2U) { status.state=1U; configure(now_us); return; }
    if ((uint32_t)(now_us - (status.samples ? status.last_sample_us : started_us)) > 100000U) {
        fault(4U,0x07U); return;
    }
    if((uint32_t)(now_us-check_phase)>=1000000U) {
        uint8_t actual=0U, reg=check_index == 0U ? 0x0FU : registers[check_index-1U];
        uint8_t expected=check_index == 0U ? 0x49U : values[check_index-1U];
        ok=transfer(reg,true,0U,&actual);
        if(ok<=0) { return; }
        check_phase=now_us; check_index=(check_index+1U)%5U;
        if(actual!=expected) { status.expected=expected; status.actual=actual; fault(3U,reg); }
        return;
    }
    interval=(uint32_t)(now_us-poll_phase);
    if (interval < 4000U) { return; }
    ok=Mag_Port_Transfer(tx,rx,sizeof(tx));
    if (ok == 0) { return; }
    poll_phase += (interval/4000U)*4000U;
    if (ok < 0) { ++status.bus_errors; fault(2U,0x07U); return; }
    if ((rx[1] & 0x08U) == 0U) { ++status.not_ready; return; }
    if ((rx[1] & 0x80U) != 0U) { ++status.overruns; }
    for (i=0U;i<3U;++i) {
        uint32_t bits = (uint32_t)rx[2U+i*2U] | ((uint32_t)rx[3U+i*2U]<<8U);
        int32_t raw = bits & 0x8000U ? (int32_t)bits-65536 : (int32_t)bits;
        status.raw[i]=raw;
        /* FMUv2 LSM303D raw axes already match board FRD (PX4 no rotation).
         * ST +/-2G sensitivity: 0.080 mgauss/count = 0.008 microtesla/count. */
        latest.raw_body_uT[i]=(float)raw*0.008f;
        centered[i]=latest.raw_body_uT[i]-(status.calibration_valid ? bias[i] : 0.0f);
        if (raw <= -32760 || raw >= 32760) {
            saturated=true;
        }
    }
    status.norm_uT=0.0f;
    for(i=0U;i<3U;++i) {
        latest.field_body_uT[i]=0.0f;
        if(status.calibration_valid) {
            for(j=0U;j<3U;++j) { latest.field_body_uT[i]+=matrix[i][j]*centered[j]; }
        } else { latest.field_body_uT[i]=centered[i]; }
        status.field_uT[i]=latest.field_body_uT[i];
        status.norm_uT+=status.field_uT[i]*status.field_uT[i];
    }
    status.norm_uT=sqrtf(status.norm_uT);
    latest.timestamp_us=now_us; latest.sequence=++status.samples;
    latest.reference_uT=MAG_CALIBRATION_REFERENCE_UT;
    status.sample_valid=saturated ? 0U : 1U;
    if(saturated) { ++status.invalid_samples; }
    latest.flags=saturated ? 0U : MAG_SAMPLE_VALID | (status.calibration_valid ? MAG_SAMPLE_CALIBRATED : 0U);
    if (status.samples > 1U) {
        interval=(uint32_t)(now_us-status.last_sample_us);
        if(interval>status.max_interval_us) { status.max_interval_us=interval; }
    }
    status.last_sample_us=now_us; pending=true;
}

bool Mag_PopSample(MagSample *sample)
{
    if(!pending || sample == NULL) { return false; }
    *sample=latest; pending=false; return true;
}

void Mag_GetStatus(MagStatus *out)
{
    if(out != NULL) { *out=status; }
}
