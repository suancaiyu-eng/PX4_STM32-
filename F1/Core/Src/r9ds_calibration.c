#include "r9ds_calibration.h"
#include "r9ds_calibration_data.h"
#include <stddef.h>

void R9DS_Normalize(const uint16_t raw[10], bool input_valid, r9ds_normalized_t *out)
{
    unsigned int i;
    if (raw == NULL || out == NULL) { return; }
    out->configured_mask = R9DS_CALIBRATION_VALID_MASK;
    out->valid_mask = 0U;
    for (i = 0U; i < 10U; ++i) {
        int32_t value = raw[i];
        int32_t low = r9ds_cal_min[i], center = r9ds_cal_center[i], high = r9ds_cal_max[i];
        out->channel[i] = 0;
        if (!input_valid || (out->configured_mask & (1U << i)) == 0U || low >= center || center >= high) {
            continue;
        }
        if (value <= low) { value = -1000; }
        else if (value >= high) { value = 1000; }
        else if (value < center) { value = (value - center) * 1000 / (center - low); }
        else { value = (value - center) * 1000 / (high - center); }
        out->channel[i] = (int16_t)value;
        out->valid_mask |= (uint16_t)(1U << i);
    }
}
