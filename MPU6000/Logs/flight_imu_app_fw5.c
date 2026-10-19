#include "flight_imu_app.h"
#include "mpu6000_port.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
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
static MPU6000_Sample latest;
static bool have_sample;

void FlightImuApp_Init(void)
{
    uint32_t now;
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
    static const uint8_t boot_line[] = "#BOOT MPU6000 FW5 USART2=460800 FireWater\r\n";
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
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_FIREWATER
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
    have_sample = false;
    g_imu_sample_valid = 0U;
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
    uint32_t now, i;
    bool valid;

    MPU6000_Port_Service();
    MPU6000_Service(MPU6000_Port_NowUs());
    /* Bounded work: every acquired sample has its own timestamp. Future attitude
     * estimation belongs here, at the acquisition rate, before telemetry decimation. */
    for (i = 0U; i < MPU6000_QUEUE_CAPACITY && MPU6000_PopSample(&sample); ++i) {
        latest = sample;
        have_sample = true;
        MPU6000_Convert(&sample, &measurement);
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
    ImuTelemetry_Service(&record, now);
    ImuTelemetry_GetStats(&telemetry_status);
    g_telemetry_status = telemetry_status;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == MPU_DRDY_Pin) { MPU6000_OnDataReady(MPU6000_Port_NowUs()); }
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
