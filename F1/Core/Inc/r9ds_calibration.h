#ifndef R9DS_CALIBRATION_H
#define R9DS_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    int16_t channel[10]; /* -1000..1000, valid only if corresponding bit is set. */
    uint16_t configured_mask;
    uint16_t valid_mask;
} r9ds_normalized_t;
void R9DS_Normalize(const uint16_t raw[10], bool input_valid, r9ds_normalized_t *out);
#endif
