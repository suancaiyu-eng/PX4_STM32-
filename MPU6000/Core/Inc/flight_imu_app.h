#ifndef FLIGHT_IMU_APP_H
#define FLIGHT_IMU_APP_H

#include "mpu6000.h"
#include "imu_telemetry.h"
#include "mpu6000_port.h"
#include "attitude_estimator.h"

/* Set 0 to restore exactly the original seven FireWater raw channels.
 * The estimator still runs; binary v1 is unchanged in either setting. */
#ifndef FLIGHT_IMU_FIREWATER_ATTITUDE
#define FLIGHT_IMU_FIREWATER_ATTITUDE 1U
#endif

/* Debugger snapshots, written only by the foreground. The control loop should
 * consume queued samples, not use these asynchronously as an estimator input. */
extern volatile MPU6000_Sample g_imu_sample;
extern volatile MPU6000_Measurement g_imu_measurement;
extern volatile MPU6000_Status g_imu_status;
extern volatile ImuTelemetryStats g_telemetry_status;
extern volatile uint32_t g_imu_sample_valid;
/* 1=main, 2=HAL ready, 3=clock ready, 4=peripherals ready, 5=app ready, 6=loop */
extern volatile uint32_t g_app_boot_stage;
/* HAL_OK=0; 0xFFFFFFFF means the FireWater boot UART check has not run. */
extern volatile uint32_t g_boot_uart_result;
extern volatile MPU6000_BusProbe g_sensor_bus_probe;
extern volatile AttitudeEstimate g_attitude;
extern volatile uint32_t g_attitude_max_update_us;
extern volatile MagStatus g_mag_status;
extern volatile MagSample g_mag_sample;
extern volatile SensorSpiBusStats g_spi_bus_status;

void FlightImuApp_Init(void);
void FlightImuApp_Service(void);
#endif
