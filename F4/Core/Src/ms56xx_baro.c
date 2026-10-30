#include "ms56xx_baro.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

#define CONVERSION_US 10000U /* OSR4096 maximum 9.04ms plus margin */
#define REFERENCE_COUNT 100U
enum { PH_RESET, PH_RESET_WAIT, PH_PROM, PH_D2_START, PH_D2_WAIT,
       PH_D1_START, PH_D1_WAIT };
static BaroStatus status;
static BaroSample latest;
static uint32_t phase, phase_us, prom_index, warmup_us, conversion_us;
static uint32_t d2, reference_min, reference_max;
static uint64_t reference_sum;
static bool pending, warmup_started;

uint8_t Baro_Crc4(const uint16_t prom[8])
{
    uint16_t remainder=0U, word;
    uint32_t i,j;
    if(prom == NULL) { return 0xffU; }
    for(i=0U;i<16U;++i) {
        word=prom[i/2U];
        if(i/2U == 7U) { word &= 0xff00U; }
        remainder ^= (i & 1U) ? (word & 0xffU) : (word >> 8U);
        for(j=0U;j<8U;++j) {
            remainder=(uint16_t)((remainder & 0x8000U) ?
                ((uint32_t)remainder << 1U)^0x3000U : (uint32_t)remainder << 1U);
        }
    }
    return (uint8_t)((remainder >> 12U)&15U);
}

bool Baro_Compensate(uint32_t model, const uint16_t c[8], uint32_t d1_value,
    uint32_t d2_value, int32_t *pressure, int32_t *temperature)
{
    int64_t dt,temp,off,sens,t2=0,off2=0,sens2=0,square,cold,p;
    if(c == NULL || pressure == NULL || temperature == NULL ||
       (model != 5611U && model != 5607U) || d1_value == 0U ||
       d2_value == 0U || d1_value >= 0xffffffU || d2_value >= 0xffffffU) { return false; }
    dt=(int64_t)d2_value-(int64_t)c[5]*256;
    temp=2000+dt*c[6]/8388608;
    if(model == 5611U) {
        off=(int64_t)c[2]*65536+(int64_t)c[4]*dt/128;
        sens=(int64_t)c[1]*32768+(int64_t)c[3]*dt/256;
    } else {
        off=(int64_t)c[2]*131072+(int64_t)c[4]*dt/64;
        sens=(int64_t)c[1]*65536+(int64_t)c[3]*dt/128;
    }
    if(temp < 2000) {
        square=(temp-2000)*(temp-2000);
        t2=dt*dt/2147483648LL;
        off2=model == 5611U ? 5*square/2 : 61*square/16;
        sens2=model == 5611U ? 5*square/4 : 2*square;
        if(temp < -1500) {
            cold=(temp+1500)*(temp+1500);
            off2+=(model == 5611U ? 7 : 15)*cold;
            sens2+=model == 5611U ? 11*cold/2 : 8*cold;
        }
    }
    temp-=t2; off-=off2; sens-=sens2;
    p=((int64_t)d1_value*sens/2097152-off)/32768;
    /* Retain numerical candidates for diagnosis even when outside the model's
     * operating range. The return value gates every control-use path. */
    *pressure=(int32_t)p; *temperature=(int32_t)temp;
    if(temp < -4000 || temp > 8500 || p < 1000 ||
        p > (model == 5611U ? 120000 : 200000)) { return false; }
    return true;
}

static void fault(BaroError error)
{
    status.state=BARO_FAULT; status.error=(uint32_t)error;
    status.flags=latest.flags=0U; pending=false;
}

static int transfer(uint8_t command, uint8_t *rx, uint16_t length)
{
    uint8_t tx[4]={0U,0U,0U,0U};
    int result;
    tx[0]=command;
    result=Baro_Port_Transfer(tx,rx,length);
    if(result < 0) { ++status.bus_errors; fault(BARO_ERROR_BUS); }
    else if(result == 0) { ++status.busy_skips; }
    return result;
}

static void reset_reference(void)
{
    reference_sum=0U; reference_min=reference_max=0U;
    status.reference_samples=0U; status.reference_pressure_pa=0.0f;
    latest.flags &= ~BARO_ALTITUDE_VALID; status.flags=latest.flags;
    latest.relative_altitude_m=latest.filtered_altitude_m=0.0f;
}

void Baro_Init(uint32_t now_us)
{
    memset(&status,0,sizeof(status)); memset(&latest,0,sizeof(latest));
    status.model=BARO_SENSOR_MODEL;
    phase=PH_RESET; phase_us=now_us; prom_index=d2=0U;
    pending=warmup_started=false; warmup_us=conversion_us=now_us;
    reset_reference();
}

static void publish(uint32_t d1_value, uint32_t timestamp_us)
{
    int32_t p5611=0,p5607=0,t5611=0,t5607=0;
    bool good5611,good5607;
    uint32_t previous=latest.timestamp_us, interval;
    good5611=Baro_Compensate(5611U,status.prom,d1_value,d2,&p5611,&t5611);
    good5607=Baro_Compensate(5607U,status.prom,d1_value,d2,&p5607,&t5607);
    if((status.model == 5611U && !good5611) || (status.model == 5607U && !good5607) ||
        (!good5611 && !good5607)) { fault(BARO_ERROR_RANGE); return; }
    latest.timestamp_us=timestamp_us; latest.sequence=++status.samples;
    latest.d1=d1_value; latest.d2=d2; latest.flags=BARO_RAW_VALID;
    latest.pressure_ms5611_pa=p5611; latest.pressure_ms5607_pa=p5607;
    latest.temperature_centi_c=good5611 ? t5611 : t5607;
    latest.pressure_pa=0;
    if(status.samples > 1U) {
        interval=(uint32_t)(timestamp_us-previous);
        if(interval > status.max_interval_us) { status.max_interval_us=interval; }
    }
    status.last_sample_us=timestamp_us;
    if(!warmup_started) { warmup_started=true; warmup_us=timestamp_us; }
    if(status.model != 0U) {
        latest.pressure_pa=status.model == 5611U ? p5611 : p5607;
        latest.flags |= BARO_PRESSURE_VALID;
        if(status.reference_pressure_pa == 0.0f &&
            (uint32_t)(timestamp_us-warmup_us) >= 1000000U) {
            uint32_t p=(uint32_t)latest.pressure_pa;
            if(status.reference_samples == 0U) { reference_min=reference_max=p; }
            if(p < reference_min) { reference_min=p; }
            if(p > reference_max) { reference_max=p; }
            /* Require a quiet ~2s window; 60Pa is about 5m near sea level.
             * This is a pressure stability check, not proof of being on ground. */
            if(reference_max-reference_min > 60U) {
                ++status.reference_resets; reset_reference();
            } else {
                reference_sum+=p; ++status.reference_samples;
                if(status.reference_samples == REFERENCE_COUNT) {
                    status.reference_pressure_pa=(float)reference_sum/(float)REFERENCE_COUNT;
                }
            }
        }
        if(status.reference_pressure_pa > 0.0f) {
            float dt=(float)(uint32_t)(timestamp_us-previous)*0.000001f;
            float height=44330.0f*(1.0f-powf((float)latest.pressure_pa/status.reference_pressure_pa,0.19029496f));
            latest.relative_altitude_m=height;
            if(status.reference_samples == REFERENCE_COUNT && (status.flags & BARO_ALTITUDE_VALID)) {
                latest.filtered_altitude_m+=dt/(0.15f+dt)*(height-latest.filtered_altitude_m);
            } else { latest.filtered_altitude_m=height; }
            latest.flags |= BARO_ALTITUDE_VALID;
        }
    }
    status.flags=latest.flags; pending=true;
}

void Baro_Service(uint32_t now_us, bool imu_running)
{
    uint8_t rx[4]={0U};
    uint32_t adc,i;
    if(status.state == BARO_FAULT) { return; }
    if(!imu_running) {
        if(status.state != BARO_WAIT_IMU) { fault(BARO_ERROR_IMU); }
        return;
    }
    if(status.state == BARO_WAIT_IMU) {
        status.state=BARO_INITIALIZING; phase_us=now_us;
    }
    if((uint32_t)(now_us-phase_us) > BARO_MAX_AGE_US ||
       (status.samples && (uint32_t)(now_us-status.last_sample_us) > BARO_MAX_AGE_US)) {
        fault(BARO_ERROR_TIMEOUT); return;
    }
    switch(phase) {
    case PH_RESET:
        if(transfer(0x1eU,rx,1U) <= 0) { return; }
        phase=PH_RESET_WAIT; break;
    case PH_RESET_WAIT:
        if((uint32_t)(now_us-phase_us) < 3000U) { return; }
        phase=PH_PROM; break;
    case PH_PROM:
        if(transfer((uint8_t)(0xa0U+2U*prom_index),rx,3U) <= 0) { return; }
        status.prom[prom_index++]=(uint16_t)((uint16_t)rx[1]*256U+rx[2]);
        if(prom_index == 8U) {
            for(i=1U;i<=6U;++i) {
                if(status.prom[i] == 0U || status.prom[i] == 0xffffU) { fault(BARO_ERROR_PROM); return; }
            }
            status.crc_expected=status.prom[7]&15U; status.crc_actual=Baro_Crc4(status.prom);
            if(status.crc_expected != status.crc_actual) { fault(BARO_ERROR_CRC); return; }
            phase=PH_D2_START; status.state=BARO_RUNNING;
        }
        break;
    case PH_D2_START:
    case PH_D1_START:
        if(transfer(phase == PH_D2_START ? 0x58U : 0x48U,rx,1U) <= 0) { return; }
        conversion_us=Baro_Port_NowUs();
        phase=phase == PH_D2_START ? PH_D2_WAIT : PH_D1_WAIT;
        break;
    case PH_D2_WAIT:
    case PH_D1_WAIT:
        if((uint32_t)(now_us-conversion_us) < CONVERSION_US) { return; }
        if(transfer(0x00U,rx,4U) <= 0) { return; }
        adc=((uint32_t)rx[1]<<16U)|((uint32_t)rx[2]<<8U)|rx[3];
        if(adc == 0U || adc == 0xffffffU) { fault(BARO_ERROR_ADC); return; }
        if(phase == PH_D2_WAIT) { d2=adc; phase=PH_D1_START; }
        else {
            /* Timestamp is conservative conversion-ready time, not SPI read
             * completion. It includes ADC latency and is not an IMU timestamp. */
            publish(adc,conversion_us+CONVERSION_US); phase=PH_D2_START;
        }
        break;
    default: fault(BARO_ERROR_ADC); return;
    }
    phase_us=Baro_Port_NowUs();
}

bool Baro_PopSample(BaroSample *out)
{
    if(out == NULL || !pending) { return false; }
    (void)Baro_GetLatest(Baro_Port_NowUs(),out);
    pending=false; return true;
}

bool Baro_GetLatest(uint32_t now_us, BaroSample *out)
{
    if(out == NULL) { return false; }
    *out=latest;
    if(status.state != BARO_RUNNING || !status.samples ||
        (uint32_t)(now_us-latest.timestamp_us) > BARO_MAX_AGE_US) { out->flags=0U; }
    return (out->flags & BARO_PRESSURE_VALID) != 0U;
}

void Baro_GetStatus(BaroStatus *out) { if(out != NULL) { *out=status; } }

bool Baro_ZeroDisarmed(bool disarmed)
{
    if(!disarmed || status.state != BARO_RUNNING || status.model == 0U ||
       !(latest.flags & BARO_PRESSURE_VALID) ||
       (uint32_t)(Baro_Port_NowUs()-latest.timestamp_us) > BARO_MAX_AGE_US) { return false; }
    reset_reference(); pending=false; ++status.reference_resets; return true;
}
