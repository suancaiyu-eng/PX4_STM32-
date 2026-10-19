#ifndef MPU6000_PORT_H
#define MPU6000_PORT_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t mpu_id[3];
    uint8_t mpu_pwr1;
    uint8_t gyro_id;
    uint8_t accel_mag_id;
    uint8_t en_pin;
    uint8_t cs_low_pin;
    uint8_t cs_high_pin;
    uint8_t sck_idle_pin;
    uint8_t hal_errors; /* bits 0..5 correspond to the six register reads */
} MPU6000_BusProbe;
/* Read-only board check before acquisition starts, after >=100ms rail settling.
 * Leaves every CS high and DRDY masked. Never call during active acquisition. */
bool MPU6000_Port_Probe(MPU6000_BusProbe *result);

/* Board boundary; host tests replace these functions with a simulated sensor. */
uint32_t MPU6000_Port_NowUs(void);
void MPU6000_Port_Select(bool selected);
void MPU6000_Port_EnableDrdy(bool enable);
bool MPU6000_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length);
/* Foreground register access: 10us CS-high and 10us setup then polling SPI.
 * Always releases CS. Fails on a stopped timer or an owned DMA/SPI bus. */
bool MPU6000_Port_TransferRegister(const uint8_t *tx, uint8_t *rx, uint16_t length);
bool MPU6000_Port_StartDma(const uint8_t *tx, uint8_t *rx, uint16_t length);
void MPU6000_Port_Stop(void); /* stop requests, no waiting; safe in IRQ */
bool MPU6000_Port_Recover(void); /* explicit foreground restart, only after DMA stopped */
uint32_t MPU6000_Port_Lock(void);
void MPU6000_Port_Unlock(uint32_t key);
void MPU6000_Port_Barrier(void);
/* Board IRQ adapter avoids HAL's end-of-transfer polling inside the DMA ISR. */
/* True consumes this IRQ (fault/deferred); false lets generated HAL run. */
bool MPU6000_Port_HandleRxDmaIRQ(void);
bool MPU6000_Port_HandleTxDmaIRQ(void);
void MPU6000_Port_Service(void);

typedef struct {
    uint32_t transactions, busy_skips, errors, deferred_drdy, lost_drdy, max_transfer_us;
} SensorSpiBusStats;
/* All live DRDY events pass through this arbiter. A peer lease defers the
 * original timestamp, never starts another HAL operation on the owned SPI. */
void MPU6000_Port_OnDataReady(uint32_t timestamp_us);
/* Foreground PC15 transactions: 1=complete, 0=try later, -1=transfer failed.
 * Short polling transactions (max 8 bytes), only in the IMU's idle window.
 * IRQs remain enabled; unexpected DRDY is deferred until the lease releases. */
int MPU6000_Port_TransferMag(const uint8_t *tx, uint8_t *rx, uint16_t length);
void MPU6000_Port_GetBusStats(SensorSpiBusStats *stats);
#endif
