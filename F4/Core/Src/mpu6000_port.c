#include "mpu6000_port.h"
#include "mpu6000.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
static volatile uint8_t rx_irq_deferred;
static volatile uint8_t port_stopped;
static volatile uint8_t peer_owned, pending_drdy, have_drdy_clock;
static volatile uint32_t pending_timestamp, last_drdy_us;
static volatile SensorSpiBusStats bus_stats;

uint32_t MPU6000_Port_NowUs(void) { return __HAL_TIM_GET_COUNTER(&htim2); }
uint32_t MPU6000_Port_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}
void MPU6000_Port_Unlock(uint32_t key) { __DMB(); __set_PRIMASK(key); }
void MPU6000_Port_Barrier(void) { __DMB(); }

void MPU6000_Port_Select(bool selected)
{
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void MPU6000_Port_EnableDrdy(bool enable)
{
    uint32_t key = MPU6000_Port_Lock();
    CLEAR_BIT(EXTI->IMR, MPU_DRDY_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(MPU_DRDY_Pin);
    if (enable) { SET_BIT(EXTI->IMR, MPU_DRDY_Pin); }
    MPU6000_Port_Unlock(key);
}

bool MPU6000_Port_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    /* Only used by the staged initialization while DRDY acquisition is masked. */
    return HAL_SPI_TransmitReceive(&hspi1, tx, rx, length, 2U) == HAL_OK;
}

static bool register_delay_10us(void)
{
    uint32_t start = MPU6000_Port_NowUs();
    uint32_t guard;
    /* Board-tested CS spacing/setup delay. Initialization only; never used by
     * the DMA acquisition ISR. A stopped TIM2 hits the bounded failure path. */
    for (guard = 0U; guard < 10000U; ++guard) {
        if ((uint32_t)(MPU6000_Port_NowUs() - start) >= 10U) { return true; }
    }
    return false;
}

bool MPU6000_Port_TransferRegister(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    bool ok;
    if (tx == 0 || rx == 0 || length == 0U || peer_owned || hspi1.State != HAL_SPI_STATE_READY ||
        (DMA2_Stream0->CR & DMA_SxCR_EN) || (DMA2_Stream3->CR & DMA_SxCR_EN)) {
        return false;
    }
    MPU6000_Port_Select(false);
    if (!register_delay_10us()) { return false; }
    MPU6000_Port_Select(true);
    if (!register_delay_10us()) { MPU6000_Port_Select(false); return false; }
    ok = MPU6000_Port_Transfer(tx, rx, length);
    MPU6000_Port_Select(false);
    return ok;
}

void MPU6000_Port_OnDataReady(uint32_t timestamp_us)
{
    last_drdy_us = timestamp_us;
    have_drdy_clock = 1U;
    if (peer_owned) {
        if (pending_drdy == 0U) { pending_timestamp = timestamp_us; }
        if (pending_drdy < 255U) { ++pending_drdy; }
        ++bus_stats.deferred_drdy;
        return;
    }
    MPU6000_OnDataReady(timestamp_us);
}

int MPU6000_Port_TransferMag(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    uint32_t key, started, elapsed, phase;
    bool ok;
    uint8_t deferred;
    if (tx == 0 || rx == 0 || length == 0U || length > 8U) { return -1; }
    key = MPU6000_Port_Lock();
    started = MPU6000_Port_NowUs();
    phase = (uint32_t)(started - last_drdy_us);
    if (port_stopped || peer_owned || rx_irq_deferred || hspi1.State != HAL_SPI_STATE_READY ||
        (SPI1->SR & SPI_SR_BSY) || (DMA2_Stream0->CR & DMA_SxCR_EN) ||
        (DMA2_Stream3->CR & DMA_SxCR_EN) ||
        (have_drdy_clock && (phase < 230U || phase > 650U))) {
        ++bus_stats.busy_skips;
        MPU6000_Port_Unlock(key);
        return 0;
    }
    peer_owned = 1U;
    pending_drdy = 0U;
    MPU6000_Port_Unlock(key);
    /* Keep the same board-tested setup protection. IRQs are NOT masked here;
     * DRDY is timestamped and deferred by the arbiter if it arrives early. */
    ok = register_delay_10us();
    if (ok) {
        HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port, ACCEL_MAG_CS_Pin, GPIO_PIN_RESET);
        ok = register_delay_10us();
        if (ok) { ok = HAL_SPI_TransmitReceive(&hspi1, tx, rx, length, 2U) == HAL_OK; }
    }
    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port, ACCEL_MAG_CS_Pin, GPIO_PIN_SET);
    elapsed = (uint32_t)(MPU6000_Port_NowUs() - started);
    if (elapsed > bus_stats.max_transfer_us) { bus_stats.max_transfer_us = elapsed; }
    ++bus_stats.transactions;
    if (!ok) { ++bus_stats.errors; }
    key = MPU6000_Port_Lock();
    deferred = pending_drdy;
    pending_drdy = 0U;
    peer_owned = 0U;
    if (deferred > 1U || (deferred && elapsed > 250U)) {
        /* Do not hide a missed sample or violate the IMU's 500us DMA deadline. */
        ++bus_stats.lost_drdy;
        MPU6000_OnSpiError();
        ok = false;
    } else if (deferred && !port_stopped) {
        MPU6000_OnDataReady(pending_timestamp);
    }
    MPU6000_Port_Unlock(key);
    return ok ? 1 : -1;
}

void MPU6000_Port_GetBusStats(SensorSpiBusStats *stats)
{
    uint32_t key = MPU6000_Port_Lock();
    if (stats != 0) { *stats = bus_stats; }
    MPU6000_Port_Unlock(key);
}

static bool probe_read(uint8_t device, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80U), 0U};
    uint8_t rx[2] = {0U, 0U};
    bool ok;
    /* Caller has deselected all devices. All commands here are register reads. */
    if (!register_delay_10us()) { *value = 0U; return false; }
    if (device == 0U) { MPU6000_Port_Select(true); }
    else if (device == 1U) { HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); }
    else { HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port, ACCEL_MAG_CS_Pin, GPIO_PIN_RESET); }
    ok = MPU6000_Port_Transfer(tx, rx, sizeof(tx));
    MPU6000_Port_Select(false);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port, ACCEL_MAG_CS_Pin, GPIO_PIN_SET);
    *value = rx[1];
    return ok;
}

bool MPU6000_Port_Probe(MPU6000_BusProbe *out)
{
    unsigned int i;
    if (out == 0 || peer_owned || hspi1.State != HAL_SPI_STATE_READY ||
        (DMA2_Stream0->CR & DMA_SxCR_EN) || (DMA2_Stream3->CR & DMA_SxCR_EN)) {
        return false;
    }
    MPU6000_Port_EnableDrdy(false);
    __HAL_SPI_ENABLE(&hspi1);
    MPU6000_Port_Select(false);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port, ACCEL_MAG_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    out->hal_errors = 0U;
    out->en_pin = (uint8_t)HAL_GPIO_ReadPin(SENSORS_EN_GPIO_Port, SENSORS_EN_Pin);
    out->sck_idle_pin = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
    out->cs_high_pin = (uint8_t)HAL_GPIO_ReadPin(MPU_CS_GPIO_Port, MPU_CS_Pin);
    MPU6000_Port_Select(true);
    out->cs_low_pin = (uint8_t)HAL_GPIO_ReadPin(MPU_CS_GPIO_Port, MPU_CS_Pin);
    MPU6000_Port_Select(false);
    for (i = 0U; i < 3U; ++i) {
        if (!probe_read(0U, 0x75U, &out->mpu_id[i])) { out->hal_errors |= (uint8_t)(1U << i); }
    }
    if (!probe_read(0U, 0x6BU, &out->mpu_pwr1)) { out->hal_errors |= 0x08U; }
    if (!probe_read(1U, 0x0FU, &out->gyro_id)) { out->hal_errors |= 0x10U; }
    if (!probe_read(2U, 0x0FU, &out->accel_mag_id)) { out->hal_errors |= 0x20U; }
    return true;
}

bool MPU6000_Port_StartDma(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, length);
    if (result == HAL_OK) {
        /* No half-buffer callbacks are needed for this small atomic transaction. */
        __HAL_DMA_DISABLE_IT(&hdma_spi1_rx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(&hdma_spi1_tx, DMA_IT_HT);
    }
    return result == HAL_OK;
}

void MPU6000_Port_Stop(void)
{
    port_stopped = 1U;
    /* Fault stop: no HAL polling or reinitialization in interrupt context. */
    CLEAR_BIT(EXTI->IMR, MPU_DRDY_Pin);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN |
                         SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
    __HAL_SPI_DISABLE(&hspi1);
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    rx_irq_deferred = 0U;
}

bool MPU6000_Port_Recover(void)
{
    if (peer_owned || (DMA2_Stream0->CR & DMA_SxCR_EN) || (DMA2_Stream3->CR & DMA_SxCR_EN)) {
        return false; /* don't reuse buffers while DMA still owns them */
    }
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 |
                         DMA_FLAG_TEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_FLAG_FEIF3_7 | DMA_FLAG_DMEIF3_7 |
                         DMA_FLAG_TEIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TCIF3_7);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    /* Explicit foreground recovery only, after all DMA and peer leases release.
     * This resets the MCU SPI peripheral, not sensor power/configuration. */
    if (HAL_SPI_DeInit(&hspi1) != HAL_OK || HAL_SPI_Init(&hspi1) != HAL_OK) { return false; }
    __HAL_SPI_ENABLE(&hspi1);
    port_stopped = 0U;
    have_drdy_clock = pending_drdy = 0U;
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    return true;
}

static bool spi_finished(void)
{
    return ((SPI1->SR & SPI_SR_BSY) == 0U) && ((SPI1->SR & SPI_SR_TXE) != 0U);
}

static bool dma_error_enabled(DMA_Stream_TypeDef *stream, uint32_t flags)
{
    /* Match HAL_DMA_IRQHandler: a status bit alone is not an enabled IRQ.
     * In particular, FIFO error flags must not stop a direct-mode transfer
     * when FEIE is disabled. Callers normalize stream flags to stream-0 bits. */
    return (((flags & DMA_LISR_TEIF0) != 0U) && ((stream->CR & DMA_IT_TE) != 0U)) ||
           (((flags & DMA_LISR_DMEIF0) != 0U) && ((stream->CR & DMA_IT_DME) != 0U)) ||
           (((flags & DMA_LISR_FEIF0) != 0U) && ((stream->FCR & DMA_IT_FE) != 0U));
}

static bool handle_dma_fault(void)
{
    uint32_t flags = DMA2->LISR;
    if (dma_error_enabled(DMA2_Stream0, flags) ||
        dma_error_enabled(DMA2_Stream3, flags >> 22U)) {
        /* HAL's transfer-error handler polls DMA EN. Latch and stop immediately
         * here; foreground recovery waits for ownership before touching buffers. */
        hspi1.ErrorCode |= HAL_SPI_ERROR_DMA;
        MPU6000_OnSpiError();
        return true;
    }
    return false;
}

bool MPU6000_Port_HandleTxDmaIRQ(void) { return handle_dma_fault(); }

bool MPU6000_Port_HandleRxDmaIRQ(void)
{
    uint32_t flags = DMA2->LISR;
    if (handle_dma_fault()) { return true; }
    if ((flags & DMA_LISR_TCIF0) != 0U &&
        !spi_finished()) {
        /* HAL completion normally waits for BSY. Defer to the foreground instead
         * of waiting in a high-priority ISR; MPU Service enforces a 500us deadline. */
        HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
        rx_irq_deferred = 1U;
        return true;
    }
    return false;
}

void MPU6000_Port_Service(void)
{
    if (rx_irq_deferred && spi_finished()) {
        /* RX completion shares driver state with EXTI/SPI IRQs. Block just those
         * priorities while HAL completes an already-finished transfer. */
        uint32_t key = __get_BASEPRI();
        __set_BASEPRI_MAX(5U << (8U - __NVIC_PRIO_BITS));
        __DSB(); __ISB();
        if (rx_irq_deferred && spi_finished()) {
            rx_irq_deferred = 0U;
            if (!handle_dma_fault()) { HAL_DMA_IRQHandler(&hdma_spi1_rx); }
            /* Fault callback can disable DMA IRQs; only re-enable a healthy SPI. */
            if (!port_stopped && hspi1.State == HAL_SPI_STATE_READY && hspi1.ErrorCode == HAL_SPI_ERROR_NONE) {
                HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
                HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
            }
        }
        __set_BASEPRI(key);
        __DSB(); __ISB();
    }
}
