#ifndef R9DS_TELEM_H
#define R9DS_TELEM_H
#include <stdint.h>
typedef struct {
    uint32_t sent;
    uint32_t queue_full;
} r9ds_telem_stats_t;
extern volatile r9ds_telem_stats_t g_r9ds_telem;
void R9DS_Telem_Init(void);
void R9DS_Telem_Service(void);
#endif
