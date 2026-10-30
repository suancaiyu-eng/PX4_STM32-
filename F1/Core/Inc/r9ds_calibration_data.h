#ifndef R9DS_CALIBRATION_DATA_H
#define R9DS_CALIBRATION_DATA_H
/* Measured R9DS calibration. Unmeasured channels remain invalid. */
#define R9DS_CALIBRATION_VALID_MASK 0x03BFU
static const uint16_t r9ds_cal_min[10] = {206, 200, 206, 200, 200, 200, 1800, 200, 200, 208};
static const uint16_t r9ds_cal_center[10] = {1000, 995, 1000, 996, 1000, 1000, 1800, 1000, 1000, 1008};
static const uint16_t r9ds_cal_max[10] = {1800, 1793, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1808};
#endif
