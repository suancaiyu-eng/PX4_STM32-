#include "main_pwm_port.h"
#include "main_pwm.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

TIM_TypeDef host_tim2, host_tim3, host_tim4;
GPIO_TypeDef host_gpioa, host_gpiob;
AFIO_TypeDef host_afio;
uint32_t host_irq;
static uint32_t pclk = 32000000U;
static unsigned af_switches;
void Error_Handler(void) { assert(0 && "HAL initialization failure"); }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *c, uint32_t *latency)
{ c->APB1CLKDivider = 2U; *latency = 2U; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
void HAL_GPIO_WritePin(GPIO_TypeDef *g, uint16_t pins, uint32_t value)
{ assert(value == 0); g->odr &= ~pins; }
void HAL_GPIO_Init(GPIO_TypeDef *g, const GPIO_InitTypeDef *config)
{
    unsigned i;
    for (i = 0; i < 16; ++i) {
        if ((config->Pin & (1U << i)) != 0) {
            assert((g->odr & (1U << i)) == 0); /* Latch low before mode change. */
            g->mode[i] = config->Mode;
            if (i < 8) {
                g->CRL = (g->CRL & ~(0xFU << (i * 4))) |
                         ((config->Mode == GPIO_MODE_AF_PP ? 0xBU : 3U) << (i * 4));
            } else {
                g->CRH = (g->CRH & ~(0xFU << ((i - 8) * 4))) |
                         ((config->Mode == GPIO_MODE_AF_PP ? 0xBU : 3U) << ((i - 8) * 4));
            }
        }
    }
    if (config->Mode == GPIO_MODE_AF_PP) {
        TIM_TypeDef *t = g == GPIOB ? TIM4 : (config->Pin == GPIO_PIN_1 ? TIM2 : TIM3);
        assert((t->CR1 & TIM_CR1_CEN) == 0 && t->CNT == 2499);
        assert(t->CCER != 0 && t->EGR == TIM_EGR_UG);
        ++af_switches;
    }
}
uint32_t HAL_TIM_PWM_Init(TIM_HandleTypeDef *h)
{
    assert(h->Init.Prescaler == 63 && h->Init.Period == 2499);
    assert(h->Init.AutoReloadPreload == TIM_AUTORELOAD_PRELOAD_ENABLE);
    assert(h->Instance->CCER == 0 && !(h->Instance->CR1 & TIM_CR1_CEN));
    h->Instance->PSC = h->Init.Prescaler;
    h->Instance->ARR = h->Init.Period;
    return HAL_OK;
}
uint32_t HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, const TIM_OC_InitTypeDef *oc, uint32_t channel)
{
    assert(oc->Pulse == 0 && oc->OCMode == TIM_OCMODE_PWM1 && oc->OCPolarity == TIM_OCPOLARITY_HIGH);
    assert((h->Instance == TIM2 && channel == TIM_CHANNEL_2) ||
           (h->Instance == TIM3 && channel == TIM_CHANNEL_1) ||
           (h->Instance == TIM4 && (channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4)));
    if (channel == TIM_CHANNEL_1) { h->Instance->CCMR1 |= 0x68U; }
    if (channel == TIM_CHANNEL_2) { h->Instance->CCMR1 |= 0x6800U; }
    if (channel == TIM_CHANNEL_3) { h->Instance->CCMR2 |= 0x68U; }
    if (channel == TIM_CHANNEL_4) { h->Instance->CCMR2 |= 0x6800U; }
    return HAL_OK;
}
int main(void)
{
    uint16_t pulse[4] = {1000, 1500, 2000, 1100};
    uint16_t readback[4];
    uint8_t mask;
    AFIO->MAPR = AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_TIM4_REMAP | 0x10U;
    assert(MainPwmPort_Init());
    assert(AFIO->MAPR == (AFIO_MAPR_SWJ_CFG_JTAGDISABLE | 0x10U));
    assert(GPIOA->mode[1] == GPIO_MODE_OUTPUT_PP && GPIOA->mode[6] == GPIO_MODE_OUTPUT_PP);
    assert(GPIOB->mode[8] == GPIO_MODE_OUTPUT_PP && GPIOB->mode[9] == GPIO_MODE_OUTPUT_PP);
    /* Each physical MAIN channel must work independently with its own pin
     * and timer enable bit, not only when all four are submitted together. */
    {
        unsigned channel;
        for (channel = 0; channel < 4; ++channel) {
            uint16_t single[4] = {0,0,0,0};
            single[channel] = (uint16_t)(1000 + channel * 200);
            MainPwmPort_Apply((uint8_t)(1U << (channel + 1)), single);
            MainPwmPort_GetState(&mask, readback);
            assert(mask == (1U << (channel + 1)) && readback[channel] == single[channel]);
            assert(TIM2->CCER == (channel == 0 ? TIM_CCER_CC2E : 0));
            assert(TIM4->CCER == (channel == 1 ? TIM_CCER_CC3E : channel == 2 ? TIM_CCER_CC4E : 0));
            assert(TIM3->CCER == (channel == 3 ? TIM_CCER_CC1E : 0));
            MainPwmPort_Stop();
        }
        af_switches = 0;
    }
    MainPwmPort_Apply(0x1e, pulse);
    assert(TIM2->CCER == TIM_CCER_CC2E && TIM2->CCR2 == 1000);
    assert(TIM3->CCER == TIM_CCER_CC1E && TIM3->CCR1 == 1100);
    assert(TIM4->CCER == (TIM_CCER_CC3E | TIM_CCER_CC4E));
    assert(TIM4->CCR3 == 1500 && TIM4->CCR4 == 2000 && af_switches == 3);
    MainPwmPort_GetState(&mask, readback);
    assert(mask == 0x1e && readback[2] == 2000);
    TIM4->CCER &= ~TIM_CCER_CC4E; /* State must reflect hardware, not cached frame. */
    MainPwmPort_GetState(&mask, readback);
    assert(mask == 0x16 && readback[2] == 0);
    TIM4->CCER |= TIM_CCER_CC4E;
    TIM4->ARR = 19999;
    MainPwmPort_GetState(&mask, readback);
    assert(mask == 0x12 && readback[1] == 0 && readback[2] == 0);
    TIM4->ARR = 2499;
    TIM4->CCER |= (TIM_CCER_CC4E << 1);
    MainPwmPort_GetState(&mask, readback);
    assert(mask == 0x16); /* Inverted polarity cannot be reported as normal. */
    TIM4->CCER &= ~(TIM_CCER_CC4E << 1);
    assert(!host_irq);
    TIM4->CNT = 1800;
    TIM4->EGR = 0; /* A running width update must NOT generate UG or reset CNT. */
    pulse[1] = 1100; pulse[2] = 1900;
    MainPwmPort_Apply(0x1e, pulse);
    assert(TIM4->EGR == 0 && TIM4->CNT == 1800 && af_switches == 3);
    assert(TIM4->CCR3 == 1100 && TIM4->CCR4 == 1900 && !(TIM4->CR1 & TIM_CR1_UDIS));
    pulse[0] = pulse[2] = pulse[3] = 0;
    MainPwmPort_Apply(4, pulse);
    assert(TIM2->CCER == 0 && TIM3->CCER == 0 && TIM4->CCER == TIM_CCER_CC3E);
    assert(GPIOB->mode[9] == GPIO_MODE_OUTPUT_PP);
    MainPwmPort_Stop();
    assert(TIM2->CCER == 0 && TIM3->CCER == 0 && TIM4->CCER == 0);
    assert(!(TIM2->CR1 & 1) && !(TIM3->CR1 & 1) && !(TIM4->CR1 & 1));
    assert(GPIOA->mode[0] == 0 && GPIOA->mode[7] == 0 && GPIOB->mode[0] == 0 && GPIOB->mode[1] == 0);
    pclk = 36000000U;
    assert(!MainPwmPort_Init() && TIM4->CCER == 0);
    puts("MAIN PWM STM32 adapter: PASS (register stubs; NOT a waveform measurement)");
    return 0;
}
