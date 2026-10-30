#include "r9ds_telem.h"
#include "r9ds.h"
#include "board_link.h"
#include "rc_link.h"

volatile r9ds_telem_stats_t g_r9ds_telem;
static uint32_t last_ms;
static uint16_t sequence;

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U); p[3] = (uint8_t)(value >> 24U);
}

void R9DS_Telem_Init(void)
{
    last_ms = HAL_GetTick();
    sequence = 0U;
    g_r9ds_telem.sent = 0U;
    g_r9ds_telem.queue_full = 0U;
}

void R9DS_Telem_Service(void)
{
    r9ds_snapshot_t snapshot;
    uint8_t bytes[RC_LINK_SIZE];
    uint32_t now = HAL_GetTick();
    unsigned int i;
    if ((uint32_t)(now - last_ms) < 50U) { return; }
    last_ms = now;
    (void)R9DS_GetSnapshot(&snapshot);
    bytes[0] = RC_LINK_VERSION;
    bytes[1] = (uint8_t)snapshot.status;
    bytes[2] = (uint8_t)((snapshot.received ? 1U : 0U) | (snapshot.input_valid ? 2U : 0U) |
        (snapshot.frame_lost ? 4U : 0U) | (snapshot.failsafe ? 8U : 0U) |
        (snapshot.digital17 ? 16U : 0U) | (snapshot.digital18 ? 32U : 0U));
    bytes[3] = RC_LINK_CHANNELS;
    put32(bytes + 4U, snapshot.last_frame_ms);
    put32(bytes + 8U, snapshot.age_ms);
    put32(bytes + 12U, snapshot.counters.frames_received);
    put32(bytes + 16U, snapshot.counters.uart_errors);
    put32(bytes + 20U, snapshot.counters.invalid_frames);
    put32(bytes + 24U, snapshot.counters.queue_drops);
    put32(bytes + 28U, snapshot.counters.buffer_overflows);
    put32(bytes + 32U, snapshot.counters.frame_lost_reports);
    put32(bytes + 36U, snapshot.counters.failsafe_reports);
    for (i = 0U; i < RC_LINK_CHANNELS; ++i) {
        bytes[40U + i * 2U] = (uint8_t)snapshot.channel[i];
        bytes[41U + i * 2U] = (uint8_t)(snapshot.channel[i] >> 8U);
    }
    if (BoardLink_Send(BL_KIND_EVENT, BL_CMD_RC_INPUT, (uint16_t)(sequence + 1U), bytes, sizeof(bytes))) {
        sequence++;
        g_r9ds_telem.sent++;
    } else {
        g_r9ds_telem.queue_full++;
    }
}
