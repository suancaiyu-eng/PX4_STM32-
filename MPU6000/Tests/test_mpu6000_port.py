"""Exercise the real STM32 port's boot probe and interrupt dispatch.

This covers dispatch/ownership decisions, not silicon timing or HAL internals.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#ifndef FAKE_HAL_H
#define FAKE_HAL_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint32_t CR, FCR; } DMA_Stream_TypeDef;
typedef struct { uint32_t CR2, SR; } SPI_TypeDef;
typedef struct { uint32_t LISR; } DMA_TypeDef;
typedef struct { uint32_t IMR; } EXTI_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; } DMA_HandleTypeDef;
typedef struct { uint32_t State, ErrorCode; } SPI_HandleTypeDef;
typedef struct { uint32_t counter; } TIM_HandleTypeDef;
extern SPI_TypeDef spi_regs;
extern DMA_TypeDef dma_regs;
extern EXTI_TypeDef exti_regs;
extern DMA_Stream_TypeDef rx_regs, tx_regs;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
#define SPI1 (&spi_regs)
#define DMA2 (&dma_regs)
#define EXTI (&exti_regs)
#define DMA2_Stream0 (&rx_regs)
#define DMA2_Stream3 (&tx_regs)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define MPU_CS_GPIO_Port 0
#define MPU_CS_Pin 4
#define GYRO_CS_GPIO_Port 0
#define GYRO_CS_Pin 8
#define ACCEL_MAG_CS_GPIO_Port 0
#define ACCEL_MAG_CS_Pin 16
#define BARO_CS_GPIO_Port 0
#define BARO_CS_Pin 32
#define SENSORS_EN_GPIO_Port 0
#define SENSORS_EN_Pin 64
#define GPIOA 0
#define GPIO_PIN_5 128
#define MPU_DRDY_Pin (1U << 15)
#define __NVIC_PRIO_BITS 4
#define HAL_OK 0
#define HAL_SPI_ERROR_NONE 0
#define HAL_SPI_ERROR_DMA 8
#define HAL_SPI_STATE_READY 1
#define DMA_IT_HT 16
#define DMA_IT_TE 4U
#define DMA_IT_DME 2U
#define DMA_IT_FE 128U
#define DMA_SxCR_EN 1
#define SPI_CR2_RXDMAEN 1
#define SPI_CR2_TXDMAEN 2
#define SPI_CR2_ERRIE 4
#define SPI_CR2_RXNEIE 8
#define SPI_CR2_TXEIE 16
#define SPI_SR_BSY 128
#define SPI_SR_TXE 2
#define DMA_LISR_FEIF0 1U
#define DMA_LISR_DMEIF0 4U
#define DMA_LISR_TEIF0 8U
#define DMA_LISR_HTIF0 16U
#define DMA_LISR_TCIF0 32U
#define DMA_LISR_FEIF3 (1U << 22)
#define DMA_LISR_DMEIF3 (4U << 22)
#define DMA_LISR_TEIF3 (8U << 22)
#define DMA_FLAG_FEIF0_4 DMA_LISR_FEIF0
#define DMA_FLAG_DMEIF0_4 DMA_LISR_DMEIF0
#define DMA_FLAG_TEIF0_4 DMA_LISR_TEIF0
#define DMA_FLAG_HTIF0_4 DMA_LISR_HTIF0
#define DMA_FLAG_TCIF0_4 DMA_LISR_TCIF0
#define DMA_FLAG_FEIF3_7 DMA_LISR_FEIF3
#define DMA_FLAG_DMEIF3_7 DMA_LISR_DMEIF3
#define DMA_FLAG_TEIF3_7 DMA_LISR_TEIF3
#define DMA_FLAG_HTIF3_7 (16U << 22)
#define DMA_FLAG_TCIF3_7 (32U << 22)
#define DMA2_Stream0_IRQn 0
#define DMA2_Stream3_IRQn 1
#define SPI1_IRQn 2
#define CLEAR_BIT(x, b) ((x) &= ~(b))
#define SET_BIT(x, b) ((x) |= (b))
extern unsigned timer_frozen;
#define __HAL_TIM_GET_COUNTER(t) (timer_frozen ? (t)->counter : (t)->counter++)
#define __HAL_GPIO_EXTI_CLEAR_IT(p) ((void)(p))
#define __HAL_DMA_DISABLE_IT(h, b) ((h)->Instance->CR &= ~(b))
#define __HAL_DMA_DISABLE(h) ((h)->Instance->CR &= ~DMA_SxCR_EN)
#define __HAL_DMA_CLEAR_FLAG(h, f) ((void)(h), dma_regs.LISR &= ~(f))
#define __HAL_SPI_DISABLE(h) ((void)(h))
#define __HAL_SPI_ENABLE(h) ((void)(h))
#define __HAL_RCC_SPI1_FORCE_RESET() ((void)0)
#define __HAL_RCC_SPI1_RELEASE_RESET() ((void)0)
typedef int HAL_StatusTypeDef;
extern unsigned irq_enabled[3], hal_calls, error_calls, stop_on_completion;
extern uint32_t primask, basepri;
static inline uint32_t __get_PRIMASK(void) { return primask; }
static inline void __disable_irq(void) { primask = 1; }
static inline void __set_PRIMASK(uint32_t x) { primask = x; }
static inline void __DMB(void) {}
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline uint32_t __get_BASEPRI(void) { return basepri; }
static inline void __set_BASEPRI(uint32_t x) { basepri = x; }
static inline void __set_BASEPRI_MAX(uint32_t x) { if (!basepri || x < basepri) basepri = x; }
void HAL_GPIO_WritePin(int p, int n, int v);
int HAL_GPIO_ReadPin(int p, int n);
static inline void HAL_NVIC_DisableIRQ(unsigned i) { irq_enabled[i] = 0; }
static inline void HAL_NVIC_EnableIRQ(unsigned i) { irq_enabled[i] = 1; }
static inline void HAL_NVIC_ClearPendingIRQ(unsigned i) { (void)i; }
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *t, uint8_t *r, uint16_t n, unsigned w);
static inline int HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, const uint8_t *t, uint8_t *r, uint16_t n)
{ (void)h; (void)t; (void)r; (void)n; return HAL_OK; }
static inline int HAL_SPI_DeInit(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline int HAL_SPI_Init(SPI_HandleTypeDef *h) { h->State = HAL_SPI_STATE_READY; h->ErrorCode = 0; return HAL_OK; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h);
#endif
'''

HARNESS = r'''
#include "fake_hal.h"
#include "mpu6000_port.h"
#include <assert.h>
#include <string.h>
SPI_TypeDef spi_regs;
DMA_TypeDef dma_regs;
EXTI_TypeDef exti_regs;
DMA_Stream_TypeDef rx_regs, tx_regs;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_spi1_rx = {&rx_regs}, hdma_spi1_tx = {&tx_regs};
unsigned irq_enabled[3], hal_calls, error_calls, stop_on_completion;
unsigned timer_frozen;
uint32_t primask, basepri;
#define ALL_CS (MPU_CS_Pin | GYRO_CS_Pin | ACCEL_MAG_CS_Pin | BARO_CS_Pin)
static unsigned gpio_levels, probe_reads, fail_read;
static unsigned freeze_on_select, selected_at;
static unsigned peer_mode, peer_inject, peer_fail, deferred_calls;
static uint32_t deferred_timestamp;
void HAL_GPIO_WritePin(int p, int n, int v)
{
    (void)p;
    if (v == GPIO_PIN_SET) gpio_levels |= (unsigned)n;
    else gpio_levels &= ~(unsigned)n;
    if (n == MPU_CS_Pin && v == GPIO_PIN_RESET) {
        selected_at = htim2.counter;
        if (freeze_on_select) timer_frozen = 1;
    }
}
int HAL_GPIO_ReadPin(int p, int n)
{
    (void)p;
    return (gpio_levels & (unsigned)n) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *t, uint8_t *r, uint16_t n, unsigned w)
{
    unsigned selected = (~gpio_levels) & ALL_CS;
    if(peer_mode) {
        MPU6000_BusProbe p;
        uint8_t reply[2];
        assert(h == &hspi1 && n <= 8 && w == 2 && selected == ACCEL_MAG_CS_Pin);
        assert(primask == 0); /* never mask IRQs during the transfer */
        assert(!MPU6000_Port_Recover());
        assert(!MPU6000_Port_Probe(&p));
        assert(!MPU6000_Port_TransferRegister(t,reply,2));
        if(peer_inject) MPU6000_Port_OnDataReady(123456U);
        if(peer_inject > 1) MPU6000_Port_OnDataReady(124456U);
        memset(r,0,n);
        return peer_fail ? 1 : HAL_OK;
    }
    assert(h == &hspi1 && n == 2 && w == 2 && t[1] == 0);
    assert((t[0] & 0x80U) != 0); /* probe must never write a register */
    assert((exti_regs.IMR & MPU_DRDY_Pin) == 0);
    ++probe_reads;
    if (probe_reads == fail_read) return 1;
    if (selected == MPU_CS_Pin) {
        assert(t[0] == 0xF5 || t[0] == 0xEB);
        r[1] = t[0] == 0xF5 ? 0x68 : 0x40;
    } else if (selected == GYRO_CS_Pin) {
        assert(t[0] == 0x8F);
        r[1] = 0xD4;
    } else {
        assert(selected == ACCEL_MAG_CS_Pin && t[0] == 0x8F);
        r[1] = 0x49;
    }
    return HAL_OK;
}
void MPU6000_OnSpiError(void) { ++error_calls; MPU6000_Port_Stop(); }
void MPU6000_OnDataReady(uint32_t timestamp) {
    assert((gpio_levels & ACCEL_MAG_CS_Pin) != 0); /* peer CS must be released */
    ++deferred_calls; deferred_timestamp=timestamp;
}
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h)
{
    assert(h == &hdma_spi1_rx);
    ++hal_calls;
    hspi1.State = HAL_SPI_STATE_READY;
    hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
    if (stop_on_completion) MPU6000_Port_Stop();
}
static void reset(void)
{
    rx_regs.CR = tx_regs.CR = 0;
    MPU6000_Port_Stop();
    assert(MPU6000_Port_Recover());
    rx_regs.CR = tx_regs.CR = DMA_IT_TE | DMA_IT_DME;
    rx_regs.FCR = tx_regs.FCR = 0;
    dma_regs.LISR = DMA_LISR_TCIF0;
    spi_regs.SR = SPI_SR_TXE;
    stop_on_completion = hal_calls = error_calls = 0;
    primask = basepri = 0;
    gpio_levels = ALL_CS | SENSORS_EN_Pin | GPIO_PIN_5;
    probe_reads = fail_read = 0;
    timer_frozen = 0;
    freeze_on_select = selected_at = 0;
    htim2.counter = 0;
    exti_regs.IMR = 0;
    peer_mode=peer_inject=peer_fail=deferred_calls=0;
    deferred_timestamp=0;
}
int main(void)
{
    MPU6000_BusProbe probe;
    const uint8_t command[2] = {0xF5U, 0U};
    uint8_t response[2] = {0U, 0U};
    reset();
    assert(MPU6000_Port_TransferRegister(command, response, 2));
    assert(response[1] == 0x68 && htim2.counter >= 20 && probe_reads == 1);
    assert(selected_at >= 10 && htim2.counter - selected_at >= 10);
    assert((gpio_levels & ALL_CS) == ALL_CS);
    reset(); peer_mode=1; peer_inject=1;
    assert(MPU6000_Port_TransferMag(command,response,2) == 1);
    assert(deferred_calls == 1 && deferred_timestamp == 123456U);
    assert((gpio_levels & ALL_CS) == ALL_CS);
    reset(); peer_mode=1; peer_inject=2;
    assert(MPU6000_Port_TransferMag(command,response,2) == -1);
    assert(error_calls == 1 && deferred_calls == 0);
    reset(); peer_mode=1;
    MPU6000_Port_OnDataReady(100U); htim2.counter=200U;
    assert(MPU6000_Port_TransferMag(command,response,2) == 0); /* outside idle window */
    htim2.counter=400U;
    assert(MPU6000_Port_TransferMag(command,response,2) == 1);
    rx_regs.CR |= DMA_SxCR_EN;
    assert(MPU6000_Port_TransferMag(command,response,2) == 0);
    reset(); peer_mode=1; peer_fail=1;
    assert(MPU6000_Port_TransferMag(command,response,2) == -1);
    assert((gpio_levels & ALL_CS) == ALL_CS);
    reset(); peer_mode=1; timer_frozen=1;
    assert(MPU6000_Port_TransferMag(command,response,2) == -1);
    assert((gpio_levels & ALL_CS) == ALL_CS);
    reset();
    timer_frozen = 1;
    assert(!MPU6000_Port_TransferRegister(command, response, 2));
    assert(probe_reads == 0 && (gpio_levels & ALL_CS) == ALL_CS);
    reset();
    freeze_on_select = 1;
    assert(!MPU6000_Port_TransferRegister(command, response, 2));
    assert(probe_reads == 0 && (gpio_levels & ALL_CS) == ALL_CS);
    reset();
    rx_regs.CR |= DMA_SxCR_EN;
    assert(!MPU6000_Port_TransferRegister(command, response, 2) && probe_reads == 0);
    reset();
    exti_regs.IMR = MPU_DRDY_Pin;
    assert(MPU6000_Port_Probe(&probe));
    assert(probe_reads == 6 && probe.hal_errors == 0);
    assert(probe.mpu_id[0] == 0x68 && probe.mpu_id[1] == 0x68 && probe.mpu_id[2] == 0x68);
    assert(probe.mpu_pwr1 == 0x40 && probe.gyro_id == 0xD4 && probe.accel_mag_id == 0x49);
    assert(probe.en_pin == 1 && probe.cs_low_pin == 0 && probe.cs_high_pin == 1 && probe.sck_idle_pin == 1);
    assert((gpio_levels & ALL_CS) == ALL_CS && !(exti_regs.IMR & MPU_DRDY_Pin));

    reset();
    fail_read = 5; /* failure on the peer gyro must release CS and be reported */
    assert(MPU6000_Port_Probe(&probe));
    assert(probe_reads == 6 && probe.hal_errors == 0x10 && probe.gyro_id == 0);
    assert(probe.accel_mag_id == 0x49 && (gpio_levels & ALL_CS) == ALL_CS);

    reset();
    assert(!MPU6000_Port_Probe(0));
    rx_regs.CR |= DMA_SxCR_EN;
    assert(!MPU6000_Port_Probe(&probe));
    rx_regs.CR &= ~DMA_SxCR_EN;
    tx_regs.CR |= DMA_SxCR_EN;
    assert(!MPU6000_Port_Probe(&probe));
    tx_regs.CR &= ~DMA_SxCR_EN;
    hspi1.State = 2; /* active transfer owns the bus */
    assert(!MPU6000_Port_Probe(&probe) && probe_reads == 0);

    reset();
    assert(!MPU6000_Port_HandleRxDmaIRQ()); /* finished: generated HAL runs */
    assert(!MPU6000_Port_HandleTxDmaIRQ());
    spi_regs.SR |= SPI_SR_BSY;
    assert(MPU6000_Port_HandleRxDmaIRQ());
    assert(!irq_enabled[0]);
    MPU6000_Port_Service();
    assert(hal_calls == 0); /* BSY never causes foreground polling either */
    spi_regs.SR &= ~SPI_SR_BSY;
    MPU6000_Port_Service();
    assert(hal_calls == 1 && irq_enabled[0] && basepri == 0);
    MPU6000_Port_Service();
    assert(hal_calls == 1);

    /* FIFO status is not an enabled error when FEIE is off (direct mode). */
    reset();
    dma_regs.LISR |= DMA_LISR_FEIF0 | DMA_LISR_FEIF3;
    assert(!MPU6000_Port_HandleRxDmaIRQ());
    assert(!MPU6000_Port_HandleTxDmaIRQ() && error_calls == 0);
    spi_regs.SR |= SPI_SR_BSY;
    assert(MPU6000_Port_HandleRxDmaIRQ()); /* still defer BSY despite masked FE */
    spi_regs.SR &= ~SPI_SR_BSY;
    MPU6000_Port_Service();
    assert(hal_calls == 1 && irq_enabled[0] && error_calls == 0);

    reset();
    dma_regs.LISR |= DMA_LISR_FEIF3;
    tx_regs.FCR = DMA_IT_FE;
    assert(MPU6000_Port_HandleTxDmaIRQ() && error_calls == 1);

    reset();
    dma_regs.LISR |= DMA_LISR_TEIF0 | DMA_LISR_DMEIF3;
    rx_regs.CR &= ~DMA_IT_TE;
    tx_regs.CR &= ~DMA_IT_DME;
    assert(!MPU6000_Port_HandleRxDmaIRQ() && error_calls == 0);

    reset();
    spi_regs.SR |= SPI_SR_BSY;
    assert(MPU6000_Port_HandleRxDmaIRQ());
    spi_regs.SR &= ~SPI_SR_BSY;
    stop_on_completion = 1; /* late/invalid completed sample latches fault */
    MPU6000_Port_Service();
    assert(hal_calls == 1 && !irq_enabled[0] && !irq_enabled[1]);

    reset();
    dma_regs.LISR |= DMA_LISR_TEIF0;
    assert(MPU6000_Port_HandleRxDmaIRQ());
    assert(error_calls == 1 && hal_calls == 0 && !irq_enabled[0]);
    reset();
    dma_regs.LISR |= DMA_LISR_DMEIF3;
    assert(MPU6000_Port_HandleTxDmaIRQ());
    assert(error_calls == 1 && hal_calls == 0 && !irq_enabled[1]);

    reset();
    spi_regs.SR |= SPI_SR_BSY;
    assert(MPU6000_Port_HandleRxDmaIRQ());
    dma_regs.LISR |= DMA_LISR_TEIF3;
    spi_regs.SR &= ~SPI_SR_BSY;
    MPU6000_Port_Service();
    assert(error_calls == 1 && hal_calls == 0 && !irq_enabled[0]);

    reset();
    MPU6000_Port_Stop();
    rx_regs.CR |= DMA_SxCR_EN; /* delayed hardware stream disable */
    assert(!MPU6000_Port_Recover() && !irq_enabled[0]);
    rx_regs.CR = 0;
    assert(MPU6000_Port_Recover() && irq_enabled[0]);
    assert(primask == 0 && basepri == 0);
    return 0;
}
'''


class PortTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("gcc"), "native gcc required")
    def test_boot_probe_irq_deferral_fault_and_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "fake_hal.h").write_text(STUB, encoding="utf-8")
            for name in ("main.h", "spi.h", "tim.h"):
                (path / name).write_text('#include "fake_hal.h"\n', encoding="utf-8")
            (path / "harness.c").write_text(HARNESS, encoding="utf-8")
            executable = path / "test_port.exe"
            compiled = subprocess.run(
                [shutil.which("gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-I", str(path), "-I", str(ROOT / "Core/Inc"),
                 str(ROOT / "Core/Src/mpu6000_port.c"), str(path / "harness.c"),
                 "-o", str(executable)], capture_output=True, text=True, timeout=30)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
