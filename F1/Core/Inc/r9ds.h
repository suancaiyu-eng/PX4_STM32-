#ifndef F1_R9DS_H
#define F1_R9DS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sbus.h"

#define R9DS_TIMEOUT_MS 100U

typedef struct {
    uint32_t frames_received;       /* Structurally valid frames, including failsafe. */
    uint32_t invalid_frames;        /* Length/header/footer/flags rejected. */
    uint32_t sync_discards;         /* Segments discarded while acquiring a boundary. */
    uint32_t buffer_overflows;
    uint32_t queue_drops;
    uint32_t uart_errors;           /* IRQs reporting one or more UART errors. */
    uint32_t parity_errors;
    uint32_t framing_errors;
    uint32_t noise_errors;
    uint32_t overrun_errors;
    uint32_t restart_errors;
    uint32_t frame_lost_reports;
    uint32_t failsafe_reports;
} r9ds_counters_t;

typedef struct {
    uint16_t channel[SBUS_CHANNEL_COUNT]; /* 0..2047 raw values, not PWM microseconds. */
    uint8_t channel_count;                /* R9DS: CH1..CH10; array is zero-based. */
    bool digital17;
    bool digital18;
    bool frame_lost;
    bool failsafe;
    bool received;
    bool input_valid;                    /* True only when status == SBUS_OK. */
    sbus_status_t status;
    uint32_t last_frame_ms;               /* IRQ receipt time, not decode time. */
    uint32_t age_ms;                      /* UINT32_MAX before the first valid frame. */
    r9ds_counters_t counters;
} r9ds_snapshot_t;

/* Call after MX_USART3_UART_Init(), once. A failed receive start is retried by
 * Process. Init/Process/GetSnapshot belong to the same foreground context.
 * GetSnapshot fills out even before first reception; false means no frame yet.
 * A true return does NOT imply healthy control input: inspect input_valid.
 */
HAL_StatusTypeDef R9DS_Init(void);
void R9DS_Process(void);
bool R9DS_GetSnapshot(r9ds_snapshot_t *out);

/* USART3 IRQ precheck. True means the IRQ was handled; skip the HAL handler. */
bool R9DS_UART_CheckError(void);

#ifdef __cplusplus
}
#endif
#endif
