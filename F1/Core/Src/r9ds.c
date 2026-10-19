#include "r9ds.h"
#include "usart.h"
#include <string.h>

#define RX_BUFFER_SIZE 32U
#define FRAME_QUEUE_SIZE 4U
#define UART_RX_ERRORS (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)

typedef struct {
    uint8_t bytes[SBUS_FRAME_SIZE];
    uint32_t received_ms;
} queued_frame_t;

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static queued_frame_t frame_queue[FRAME_QUEUE_SIZE];
static volatile uint32_t queue_write;
static volatile uint32_t queue_read;
static volatile bool synchronized;
static volatile bool restart_pending;
static volatile bool initialized;
static volatile r9ds_counters_t counters;
/* Only foreground code owns the decoder and its published channel state. */
static sbus_receiver_t receiver;

/* Caller either runs inside USART3 IRQ or masks interrupts. */
static HAL_StatusTypeDef start_receive(void)
{
    HAL_StatusTypeDef result;

    result = HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, RX_BUFFER_SIZE);
    restart_pending = (result != HAL_OK);
    if (restart_pending) {
        synchronized = false;
        counters.restart_errors++;
    }
    return result;
}

static void recover_receive(uint32_t errors)
{
    synchronized = false;
    counters.uart_errors++;
    if ((errors & USART_SR_PE) != 0U) {
        counters.parity_errors++;
    }
    if ((errors & USART_SR_FE) != 0U) {
        counters.framing_errors++;
    }
    if ((errors & USART_SR_NE) != 0U) {
        counters.noise_errors++;
    }
    if ((errors & USART_SR_ORE) != 0U) {
        counters.overrun_errors++;
    }
    /* No DMA is used: AbortReceive is synchronous and does not wait for data.
     * Reading SR then DR clears all four error flags and the stale IDLE flag.
     */
    (void)HAL_UART_AbortReceive(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)start_receive();
}

HAL_StatusTypeDef R9DS_Init(void)
{
    HAL_StatusTypeDef result;
    uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    initialized = false;
    (void)HAL_UART_AbortReceive(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    memset((void *)&counters, 0, sizeof(counters));
    sbus_receiver_init(&receiver);
    queue_write = 0U;
    queue_read = 0U;
    synchronized = false;
    restart_pending = false;
    initialized = true;
    result = start_receive();
    __set_PRIMASK(saved_primask);
    return result;
}

bool R9DS_UART_CheckError(void)
{
    uint32_t errors;

    if (!initialized) {
        return true;
    }
    errors = huart3.Instance->SR & UART_RX_ERRORS;
    if (errors != 0U) {
        /* Do this before HAL_UART_IRQHandler: HAL may otherwise finish the
         * buffer and invoke RxEventCallback before its ErrorCallback.
         */
        recover_receive(errors);
        return true;
    }
    return false;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint32_t received_ms;

    if ((huart != &huart3) || !initialized) {
        return;
    }
    if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        /* HAL's error path may invoke this callback before ErrorCallback.
         * Leave recovery/rearming to that error callback, and publish nothing.
         */
        synchronized = false;
        return;
    }
    received_ms = HAL_GetTick();
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE) {
        /* Buffer full without IDLE: reject this entire burst, including its
         * remaining tail. Only a later IDLE can restore synchronization.
         */
        counters.buffer_overflows++;
        synchronized = false;
    } else if (!synchronized) {
        counters.sync_discards++;
        synchronized = true;
    } else if (size != SBUS_FRAME_SIZE) {
        /* This IDLE is already a fresh boundary for the next segment. */
        counters.invalid_frames++;
    } else if ((uint32_t)(queue_write - queue_read) >= FRAME_QUEUE_SIZE) {
        counters.queue_drops++;
    } else {
        queued_frame_t *frame = &frame_queue[queue_write % FRAME_QUEUE_SIZE];
        memcpy(frame->bytes, rx_buffer, SBUS_FRAME_SIZE);
        frame->received_ms = received_ms;
        __DMB();
        queue_write++;
    }
    (void)start_receive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t errors = 0U;

    if ((huart != &huart3) || !initialized) {
        return;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U) {
        errors |= USART_SR_PE;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U) {
        errors |= USART_SR_FE;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U) {
        errors |= USART_SR_NE;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) {
        errors |= USART_SR_ORE;
    }
    recover_receive(errors);
}

void R9DS_Process(void)
{
    queued_frame_t frame;
    uint32_t saved_primask;
    unsigned int processed;
    bool available;

    if (!initialized) {
        return;
    }
    saved_primask = __get_PRIMASK();
    __disable_irq();
    if (restart_pending) {
        (void)HAL_UART_AbortReceive(&huart3);
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        (void)start_receive();
    }
    __set_PRIMASK(saved_primask);

    /* Bounded work, even if input continues while the main loop is decoding. */
    for (processed = 0U; processed < FRAME_QUEUE_SIZE; ++processed) {
        saved_primask = __get_PRIMASK();
        __disable_irq();
        available = (queue_read != queue_write);
        if (available) {
            frame = frame_queue[queue_read % FRAME_QUEUE_SIZE];
            queue_read++;
        }
        __set_PRIMASK(saved_primask);
        if (!available) {
            break;
        }
        if (sbus_receiver_accept(&receiver, frame.bytes, SBUS_FRAME_SIZE,
                                 frame.received_ms)) {
            counters.frames_received++;
        } else {
            /* This counter also has an ISR writer. Protect the increment. */
            saved_primask = __get_PRIMASK();
            __disable_irq();
            counters.invalid_frames++;
            __set_PRIMASK(saved_primask);
        }
    }
}

bool R9DS_GetSnapshot(r9ds_snapshot_t *out)
{
    uint32_t now_ms;
    uint32_t saved_primask;

    if (out == NULL) {
        return false;
    }
    now_ms = HAL_GetTick();
    memcpy(out->channel, receiver.latest.channel, sizeof(out->channel));
    out->channel_count = R9DS_CHANNEL_COUNT;
    out->digital17 = receiver.latest.digital17;
    out->digital18 = receiver.latest.digital18;
    out->frame_lost = receiver.latest.frame_lost;
    out->failsafe = receiver.latest.failsafe;
    out->received = receiver.received;
    out->status = sbus_receiver_status(&receiver, now_ms, R9DS_TIMEOUT_MS);
    out->input_valid = (out->status == SBUS_OK);
    out->last_frame_ms = receiver.last_frame_ms;
    out->age_ms = receiver.received ? (uint32_t)(now_ms - receiver.last_frame_ms)
                                    : UINT32_MAX;
    saved_primask = __get_PRIMASK();
    __disable_irq();
    out->counters = counters;
    __set_PRIMASK(saved_primask);
    return out->received;
}
