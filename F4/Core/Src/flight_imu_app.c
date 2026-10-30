#include "flight_imu_app.h"
#include "mpu6000_port.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#if BARO_TELEMETRY_ENABLED || (IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER && !IMU_TELEMETRY_RC_ONLY)
#include <stdio.h>
#endif

volatile MPU6000_Sample g_imu_sample;
volatile MPU6000_Measurement g_imu_measurement;
volatile MPU6000_Status g_imu_status;
volatile ImuTelemetryStats g_telemetry_status;
volatile uint32_t g_imu_sample_valid;
volatile uint32_t g_app_boot_stage;
volatile uint32_t g_boot_uart_result = 0xFFFFFFFFU;
volatile MPU6000_BusProbe g_sensor_bus_probe;
volatile AttitudeEstimate g_attitude;
volatile uint32_t g_attitude_max_update_us;
volatile MagStatus g_mag_status;
volatile MagSample g_mag_sample;
volatile SensorSpiBusStats g_spi_bus_status;
volatile BaroStatus g_baro_status;
volatile BaroSample g_baro_sample;
volatile uint32_t g_baro_pressure_valid, g_baro_altitude_valid;
#if BARO_TELEMETRY_ENABLED && IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_SEEKFREE
static uint32_t baro_telemetry_us;
static void baro_telemetry(uint32_t now, const BaroSample *sample, const BaroStatus *status)
{
    char line[240];
    int length;
    if((uint32_t)(now-baro_telemetry_us) < 50000U) { return; }
    baro_telemetry_us=now;
    /* One fixed channel set, integer units. Unknown model exposes BOTH
     * hypotheses but pressure/height valid stay zero. Existing DMA owns UART. */
    length=snprintf(line,sizeof(line),
        "baro:%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (long)sample->pressure_ms5611_pa,(long)sample->pressure_ms5607_pa,
        (long)sample->pressure_pa,(long)sample->temperature_centi_c,
        (long)(sample->relative_altitude_m*1000.0f),(long)(sample->filtered_altitude_m*1000.0f),
        (unsigned long)sample->flags,(unsigned long)status->model,
        (unsigned long)status->state,(unsigned long)status->error,
        (unsigned long)sample->sequence,(unsigned long)(now-sample->timestamp_us),
        (unsigned long)status->reference_samples,(unsigned long)status->crc_actual);
    if(length > 0 && (size_t)length < sizeof(line)) {
        (void)ImuTelemetry_QueueText(line,(uint16_t)length);
    }
}
#endif
static MPU6000_Sample latest;
static bool have_sample;

void FlightImuApp_Init(void)
{
    uint32_t now;
    AttitudeEstimate initial_attitude;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER && !IMU_TELEMETRY_RC_ONLY
#if FLIGHT_IMU_FIREWATER_ATTITUDE
    static const uint8_t boot_line[] = "#BOOT MPU6000 FW8 USART2=460800 FireWater18 AHRS9\r\n";
#else
    static const uint8_t boot_line[] = "#BOOT MPU6000 FW8 USART2=460800 FireWater7 AHRS9\r\n";
#endif
    MPU6000_BusProbe probe = {0};
    char probe_line[128];
    int probe_length;
    /* One bounded polling transmission BEFORE acquisition / DMA ownership.
     * Distinguishes UART wiring/clock/firmware problems from DMA or sensor faults.
     * Binary builds do not emit this text. All runtime output still uses DMA. */
    g_boot_uart_result = (uint32_t)HAL_UART_Transmit(&huart2, boot_line,
                                                   sizeof(boot_line) - 1U, 20U);
#endif
    /* Fail early if clock configuration has silently changed after regeneration. */
    if (HAL_RCC_GetHCLKFreq() != 180000000U ||
        HAL_RCC_GetPCLK1Freq() != 45000000U || HAL_RCC_GetPCLK2Freq() != 90000000U ||
        HAL_TIM_Base_Start(&htim2) != HAL_OK) {
        Error_Handler();
    }
#if IMU_TELEMETRY_RC_ONLY
    /* Preserve sensor-rail settling even with boot diagnostics disabled. */
    HAL_Delay(100U);
#endif
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER && !IMU_TELEMETRY_RC_ONLY
    /* Boot diagnostic only: no estimator/control task or DMA owns SPI/UART yet.
     * Preserve the enabled rail and allow settling before any identity read. */
    HAL_Delay(100U);
    if (!MPU6000_Port_Probe(&probe)) { probe.hal_errors = 0xFFU; }
    g_sensor_bus_probe = probe;
    probe_length = snprintf(probe_line, sizeof(probe_line),
        "#PROBE mpu=%02X/%02X/%02X pwr=%02X gyro=%02X accmag=%02X en=%u csL=%u csH=%u sck=%u hal=%02X\n",
        (unsigned int)probe.mpu_id[0], (unsigned int)probe.mpu_id[1],
        (unsigned int)probe.mpu_id[2], (unsigned int)probe.mpu_pwr1,
        (unsigned int)probe.gyro_id, (unsigned int)probe.accel_mag_id,
        (unsigned int)probe.en_pin, (unsigned int)probe.cs_low_pin,
        (unsigned int)probe.cs_high_pin, (unsigned int)probe.sck_idle_pin,
        (unsigned int)probe.hal_errors);
    if (probe_length > 0 && (size_t)probe_length < sizeof(probe_line)) {
        (void)HAL_UART_Transmit(&huart2, (const uint8_t *)probe_line, (uint16_t)probe_length, 20U);
    }
#endif
    now = MPU6000_Port_NowUs();
    Mag_Init(now);
    Baro_Init(now);
    g_baro_pressure_valid=g_baro_altitude_valid=0U;
#if BARO_TELEMETRY_ENABLED && IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_SEEKFREE
    baro_telemetry_us=now;
#endif
    have_sample = false;
    g_imu_sample_valid = 0U;
    g_attitude_max_update_us = 0U;
    AttitudeEstimator_Init();
    AttitudeEstimator_GetEstimate(&initial_attitude);
    g_attitude = initial_attitude;
    MPU6000_Init(now);
    ImuTelemetry_Init(&huart2, now);
}

void FlightImuApp_Service(void)
{
    MPU6000_Sample sample;
    MPU6000_Measurement measurement;
    MPU6000_Status status;
    ImuTelemetryStats telemetry_status;
    ImuTelemetryRecord record = {0};
    AttitudeEstimate attitude;
    MagStatus mag_status;
    MagSample mag_sample;
    SensorSpiBusStats bus_status;
    BaroSample baro_sample;
    BaroStatus baro_status;
    uint32_t now, i;
    bool valid;

    MPU6000_Port_Service();
    MPU6000_Service(MPU6000_Port_NowUs());
    MPU6000_GetStatus(&status);
    Mag_Service(MPU6000_Port_NowUs(),status.state == MPU6000_RUNNING);
    if(Mag_PopSample(&mag_sample)) {
        g_mag_sample=mag_sample;
        AttitudeEstimator_SetMagneticSample(&mag_sample);
    }
    Mag_GetStatus(&mag_status);
    g_mag_status=mag_status;
    if(mag_status.state == 3U) { AttitudeEstimator_SetMagneticSample(NULL); }
    MPU6000_Port_GetBusStats(&bus_status);
    g_spi_bus_status=bus_status;
    /* Every queued sample participates in gyro prediction before decimation.
     * All estimator math is foreground-only; IMU IRQs may preempt it. */
    for (i = 0U; i < MPU6000_QUEUE_CAPACITY && MPU6000_PopSample(&sample); ++i) {
        latest = sample;
        have_sample = true;
        MPU6000_Convert(&sample, &measurement);
        {
            uint32_t started = MPU6000_Port_NowUs();
            uint32_t duration;
            AttitudeEstimator_Consume(&sample, &measurement);
            duration = (uint32_t)(MPU6000_Port_NowUs() - started);
            if (duration > g_attitude_max_update_us) { g_attitude_max_update_us = duration; }
        }
        g_imu_sample = sample;
        g_imu_measurement = measurement;
    }

    now = MPU6000_Port_NowUs();
    MPU6000_GetStatus(&status);
    /* Snapshot validity must refer to the published sample, not a newer queued one. */
    valid = have_sample && status.state == MPU6000_RUNNING &&
            (uint32_t)(now - latest.timestamp_us) <= 5000U;
    g_imu_sample_valid = valid ? 1U : 0U;
    g_imu_status = status;
    /* Service after draining IMU queue; bounded SPI accesses use the same
     * foreground lease as the magnetometer. ADC conversion waits release bus. */
    Baro_Service(now,status.state == MPU6000_RUNNING);
    now=MPU6000_Port_NowUs();
    (void)Baro_GetLatest(now,&baro_sample);
    Baro_GetStatus(&baro_status);
    g_baro_sample=baro_sample; g_baro_status=baro_status;
    g_baro_pressure_valid=(baro_sample.flags & BARO_PRESSURE_VALID) ? 1U : 0U;
    g_baro_altitude_valid=(baro_sample.flags & BARO_ALTITUDE_VALID) ? 1U : 0U;
#if BARO_TELEMETRY_ENABLED && IMU_TELEMETRY_OUTPUT_FORMAT != IMU_TELEMETRY_FORMAT_SEEKFREE
    baro_telemetry(now,&baro_sample,&baro_status);
#endif
    AttitudeEstimator_Service(now, valid);
    AttitudeEstimator_GetEstimate(&attitude);
    g_attitude = attitude;
    record.timestamp_us = latest.timestamp_us;
    record.sequence = latest.sequence;
    record.temperature_raw = latest.temperature_raw;
    for (i = 0U; i < 3U; ++i) {
        record.accel_raw[i] = latest.accel_raw[i];
        record.gyro_raw[i] = latest.gyro_raw[i];
    }
    record.flags = IMU_TELEMETRY_FLAG_SENSOR_AXES | IMU_TELEMETRY_FLAG_UNCALIBRATED;
    if (valid) { record.flags |= IMU_TELEMETRY_FLAG_SAMPLE_VALID; }
    else { record.flags |= IMU_TELEMETRY_FLAG_STALE; }
    if (status.state == MPU6000_FAULT) { record.flags |= IMU_TELEMETRY_FLAG_DRIVER_FAULT; }
    record.driver_state = (uint8_t)status.state;
    record.driver_error = (uint8_t)status.last_error;
    record.sample_drops = status.busy_drops + status.queue_drops + status.invalid_samples;
    record.spi_errors = status.spi_errors;
    record.timeouts = status.timeouts;
    record.who_am_i = status.who_am_i;
    record.bad_register = status.bad_register;
    record.readback_expected = status.readback_expected;
    record.readback_actual = status.readback_actual;
    record.drdy_count = status.drdy_count;
    record.samples = status.samples;
    record.attitude_present = FLIGHT_IMU_FIREWATER_ATTITUDE;
    record.attitude_flags = attitude.flags;
    record.attitude_timestamp_us = attitude.timestamp_us;
    record.attitude_state = attitude.state;
    record.attitude_error = attitude.error;
    record.calibration_samples = attitude.calibration_samples;
    record.calibration_restarts = attitude.calibration_restarts;
    record.mag_present=FLIGHT_IMU_FIREWATER_ATTITUDE;
    record.mag_healthy=(mag_status.state == 2U && mag_status.sample_valid && mag_status.samples != 0U &&
        (uint32_t)(now-mag_status.last_sample_us) <= 50000U) ? 1U : 0U;
    record.mag_calibrated=mag_status.calibration_valid;
    record.baro_pressure_pa=baro_sample.pressure_pa;
    record.baro_temperature_centi_c=baro_sample.temperature_centi_c;
    record.baro_relative_mm=(int32_t)(baro_sample.relative_altitude_m*1000.0f);
    record.baro_filtered_mm=(int32_t)(baro_sample.filtered_altitude_m*1000.0f);
    record.baro_flags=baro_sample.flags;
    record.baro_model=baro_status.model;
    record.baro_state=baro_status.state;
    record.baro_error=baro_status.error;
    record.heading_valid=(attitude.flags & ATTITUDE_HEADING_VALID) != 0U ? 1U : 0U;
    record.mag_used=(attitude.flags & ATTITUDE_MAG_USED) != 0U ? 1U : 0U;
    if(record.mag_healthy) {
        for(i=0U;i<3U;++i) { record.mag_raw_mut[i]=mag_status.raw[i]*8; }
    }
    if ((attitude.flags & ATTITUDE_VALID) != 0U) {
        /* Bounded +/-180 degree values; avoid floating printf on the wire. */
        for (i = 0U; i < 3U; ++i) {
            float milli = attitude.euler_rad[i] * 57295.779513f;
            record.attitude_mdeg[i] = (int32_t)(milli + (milli < 0.0f ? -0.5f : 0.5f));
        }
    }
    ImuTelemetry_Service(&record, now);
    ImuTelemetry_GetStats(&telemetry_status);
    g_telemetry_status = telemetry_status;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == MPU_DRDY_Pin) { MPU6000_Port_OnDataReady(MPU6000_Port_NowUs()); }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi1) { MPU6000_OnDmaComplete(MPU6000_Port_NowUs()); }
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if (spi == &hspi1) { MPU6000_OnSpiError(); }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) { ImuTelemetry_OnTxComplete(uart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) { ImuTelemetry_OnError(uart); }
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *uart) { ImuTelemetry_OnAbortComplete(uart); }
