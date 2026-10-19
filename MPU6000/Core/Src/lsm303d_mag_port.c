#include "lsm303d_mag.h"
#include "mpu6000_port.h"
int Mag_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    return MPU6000_Port_TransferMag(tx, rx, length);
}
