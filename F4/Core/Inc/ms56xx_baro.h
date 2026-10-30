#ifndef MS56XX_BARO_H
#define MS56XX_BARO_H
#include <stdbool.h>
#include <stdint.h>

/* These parts have no usable model ID. Select only after checking the actual
 * part / pressure against a known reference. PROM CRC does NOT identify it. */
#ifndef BARO_SENSOR_MODEL
#define BARO_SENSOR_MODEL 5611U /* this board: verified pressure formula, 2026-10-21 */
/* PROM C1..C6: 44490,48500,26881,25419,34231,27531; CRC4=0.
 * Guilin indoor test: MS5611 ~99839Pa, MS5607 ~199679Pa. This identifies the
 * compensation model, not a unique readable chip ID. For another board, use
 * 0 (unconfirmed), 5611 (MS5611), or 5607 (MS5607) after verification. */
#endif
#if BARO_SENSOR_MODEL != 0 && BARO_SENSOR_MODEL != 5611 && BARO_SENSOR_MODEL != 5607
#error Unsupported BARO_SENSOR_MODEL
#endif
#ifndef BARO_TELEMETRY_ENABLED
#define BARO_TELEMETRY_ENABLED 0U /* dedicated F4_BARO_TEST target enables it */
#endif
#define BARO_MAX_AGE_US 100000U
#define BARO_RAW_VALID 1U
#define BARO_PRESSURE_VALID 2U
#define BARO_ALTITUDE_VALID 4U

typedef enum { BARO_WAIT_IMU, BARO_INITIALIZING, BARO_RUNNING, BARO_FAULT } BaroState;
typedef enum { BARO_ERROR_NONE, BARO_ERROR_BUS, BARO_ERROR_PROM,
    BARO_ERROR_CRC, BARO_ERROR_ADC, BARO_ERROR_RANGE, BARO_ERROR_TIMEOUT,
    BARO_ERROR_IMU } BaroError;
typedef struct {
    uint32_t timestamp_us, sequence, flags, d1, d2;
    int32_t pressure_pa, temperature_centi_c;
    int32_t pressure_ms5611_pa, pressure_ms5607_pa;
    float relative_altitude_m, filtered_altitude_m;
} BaroSample;
typedef struct {
    uint32_t state, error, model, samples, bus_errors, busy_skips;
    uint32_t last_sample_us, max_interval_us, reference_samples, reference_resets;
    uint32_t crc_expected, crc_actual, flags;
    uint16_t prom[8];
    float reference_pressure_pa;
} BaroStatus;

/* Foreground-only. Each Service performs at most one short SPI transaction;
 * reset and OSR4096 conversion delays never block the main loop. */
void Baro_Init(uint32_t now_us);
void Baro_Service(uint32_t now_us, bool imu_running);
bool Baro_PopSample(BaroSample *out);
/* Pop is a single latest-sample mailbox (not a queue); compare sequence to
 * detect skipped samples. It also removes valid flags from stale data. */
/* Always copies latest when out != NULL; return means compensated pressure
 * is usable NOW. Altitude additionally requires BARO_ALTITUDE_VALID. */
bool Baro_GetLatest(uint32_t now_us, BaroSample *out);
void Baro_GetStatus(BaroStatus *out);
/* Caller must supply actual disarmed state. Never auto-zero during flight.
 * A latched fault needs explicit Baro_Init from a disarmed recovery path. */
bool Baro_ZeroDisarmed(bool disarmed);
uint8_t Baro_Crc4(const uint16_t prom[8]);
bool Baro_Compensate(uint32_t model, const uint16_t prom[8], uint32_t d1,
    uint32_t d2, int32_t *pressure_pa, int32_t *temperature_centi_c);
/* Compensate can leave numerical diagnostic outputs on an out-of-range false
 * result; callers must check its return before using them as measurements. */
/* Port: +1 success, 0 busy/retry without touching device, -1 error. */
int Baro_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length);
uint32_t Baro_Port_NowUs(void);
#endif
