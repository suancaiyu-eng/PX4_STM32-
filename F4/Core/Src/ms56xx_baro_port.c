#include "ms56xx_baro.h"
#include "mpu6000_port.h"
int Baro_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    return MPU6000_Port_TransferBaro(tx,rx,length);
}
uint32_t Baro_Port_NowUs(void) { return MPU6000_Port_NowUs(); }
