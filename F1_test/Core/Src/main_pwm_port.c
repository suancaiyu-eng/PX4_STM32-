#include "main_pwm_port.h"
#include "main_pwm.h"
#include "tim.h"

static uint8_t active_mask;

static void pin_mode(GPIO_TypeDef *gpio, uint16_t pins, uint32_t mode)
{
    GPIO_InitTypeDef init = {0};
    /* Latch low before changing MODER/CRL/CRH, including the stop path. */
    HAL_GPIO_WritePin(gpio, pins, GPIO_PIN_RESET);
    init.Pin = pins;
    init.Mode = mode;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(gpio, &init);
}

void MainPwmPort_Stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM2->CCER = 0U;
    TIM3->CCER = 0U;
    TIM4->CCER = 0U;
    pin_mode(GPIOA, GPIO_PIN_1 | GPIO_PIN_6, GPIO_MODE_OUTPUT_PP);
    pin_mode(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP);
    active_mask = 0U;
}

bool MainPwmPort_Init(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency, timer_clock;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    MainPwmPort_Stop();
    HAL_RCC_GetClockConfig(&clocks, &latency);
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) { timer_clock *= 2U; }
    if (timer_clock != 64000000U) { return false; }
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    /* No remap. Set SWJ explicitly: this HAL's timer-remap helpers write the
     * write-only SWJ_CFG bits as 111. Preserve SWD and release JTAG pins. */
    __HAL_RCC_AFIO_CLK_ENABLE();
    AFIO->MAPR = (AFIO->MAPR & ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP |
                                AFIO_MAPR_TIM4_REMAP | AFIO_MAPR_SWJ_CFG)) |
                 AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    MainPwmPort_Stop();
    return true;
}

static void apply_group(TIM_TypeDef *timer, GPIO_TypeDef *gpio, uint16_t pins,
                         uint8_t group_mask, uint8_t mask, const uint16_t pulse[4])
{
    uint8_t selected = mask & group_mask;
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    if (selected != (active_mask & group_mask)) {
        /* Membership changes restart this group; never shorten an unrelated
         * timer group's pulse. Outputs go low before reconfiguring the timer. */
        timer->CCER = 0U;
        timer->CR1 &= ~TIM_CR1_CEN;
        pin_mode(gpio, pins, GPIO_MODE_OUTPUT_PP);
    }
    /* UDIS prevents TIM4 CH3/CH4 latching a mixed old/new pair if rollover
     * occurs between writes. Do not issue UG while a group is running. */
    timer->CR1 |= TIM_CR1_UDIS;
    if (timer == TIM2) { timer->CCR2 = pulse[0]; }
    else if (timer == TIM3) { timer->CCR1 = pulse[3]; }
    else { timer->CCR3 = pulse[1]; timer->CCR4 = pulse[2]; }
    timer->CR1 &= ~TIM_CR1_UDIS;
    if (selected != (active_mask & group_mask) && selected != 0U) {
        uint16_t enabled_pins = 0U;
        uint32_t enables = 0U;
        /* Load ZERO active compare first: UG must not leave OCREF high before
         * the pin handover. Requested widths remain pending until rollover. */
        if (timer == TIM2) { timer->CCR2 = 0U; }
        else if (timer == TIM3) { timer->CCR1 = 0U; }
        else { timer->CCR3 = 0U; timer->CCR4 = 0U; }
        timer->EGR = TIM_EGR_UG;
        if (timer == TIM2) { timer->CCR2 = pulse[0]; }
        else if (timer == TIM3) { timer->CCR1 = pulse[3]; }
        else { timer->CCR3 = pulse[1]; timer->CCR4 = pulse[2]; }
        /* Connect while active CCR is zero. The first rollover latches the
         * requested widths and begins a FULL first pulse. No live forced UG. */
        timer->CNT = MAIN_PWM_PERIOD_US - 1U;
        timer->SR = 0U;
        if (timer == TIM2) { enabled_pins = GPIO_PIN_1; enables = TIM_CCER_CC2E; }
        else if (timer == TIM3) { enabled_pins = GPIO_PIN_6; enables = TIM_CCER_CC1E; }
        else {
            if ((selected & 0x04U) != 0U) { enabled_pins |= GPIO_PIN_8; enables |= TIM_CCER_CC3E; }
            if ((selected & 0x08U) != 0U) { enabled_pins |= GPIO_PIN_9; enables |= TIM_CCER_CC4E; }
        }
        timer->CCER = enables;
        pin_mode(gpio, enabled_pins, GPIO_MODE_AF_PP);
        timer->CR1 |= TIM_CR1_CEN;
    }
    __set_PRIMASK(saved_irq);
}

void MainPwmPort_Apply(uint8_t mask, const uint16_t pulse_us[4])
{
    apply_group(TIM2, GPIOA, GPIO_PIN_1, 0x02U, mask, pulse_us);
    apply_group(TIM4, GPIOB, GPIO_PIN_8 | GPIO_PIN_9, 0x0CU, mask, pulse_us);
    apply_group(TIM3, GPIOA, GPIO_PIN_6, 0x10U, mask, pulse_us);
    active_mask = mask;
}

void MainPwmPort_GetState(uint8_t *mask, uint16_t pulse_us[4])
{
    TIM_TypeDef *timers[4] = {TIM2, TIM4, TIM4, TIM3};
    GPIO_TypeDef *gpios[4] = {GPIOA, GPIOB, GPIOB, GPIOA};
    const uint8_t pins[4] = {1U, 8U, 9U, 6U};
    const uint8_t shifts[4] = {8U, 0U, 8U, 0U};
    const uint32_t enables[4] = {TIM_CCER_CC2E, TIM_CCER_CC3E, TIM_CCER_CC4E, TIM_CCER_CC1E};
    uint32_t compares[4];
    uint8_t i;
    compares[0] = TIM2->CCR2; compares[1] = TIM4->CCR3;
    compares[2] = TIM4->CCR4; compares[3] = TIM3->CCR1;
    *mask = 0U;
    for (i = 0U; i < 4U; ++i) {
        uint32_t cr = pins[i] < 8U ? gpios[i]->CRL : gpios[i]->CRH;
        uint32_t mode = (cr >> ((pins[i] % 8U) * 4U)) & 0xFU;
        uint32_t ccmr = (i == 1U || i == 2U) ? timers[i]->CCMR2 : timers[i]->CCMR1;
        uint32_t channel_mode = (ccmr >> shifts[i]) & 0x7BU;
        bool enabled = (timers[i]->CR1 & TIM_CR1_CEN) != 0U &&
                       (timers[i]->CCER & enables[i]) != 0U &&
                       (timers[i]->CCER & (enables[i] << 1U)) == 0U &&
                       timers[i]->PSC == 63U && timers[i]->ARR == MAIN_PWM_PERIOD_US - 1U &&
                       channel_mode == 0x68U && /* PWM1 + preload, output mode. */
                       (mode & 0xCU) == 0x8U && (mode & 3U) != 0U &&
                       compares[i] >= MAIN_PWM_MIN_US && compares[i] <= MAIN_PWM_MAX_US;
        pulse_us[i] = enabled ? (uint16_t)compares[i] : 0U;
        if (enabled) { *mask |= (uint8_t)(1U << (i + 1U)); }
    }
}
