/* CubeMX-compatible timer setup. No Start/PostInit: the PWM port owns pins. */
#include "tim.h"

TIM_HandleTypeDef htim2, htim3, htim4;

static void configure(TIM_HandleTypeDef *handle, TIM_TypeDef *instance,
                      uint32_t channel, uint32_t second_channel)
{
    TIM_OC_InitTypeDef oc = {0};
    handle->Instance = instance;
    handle->Init.Prescaler = 63U;
    handle->Init.CounterMode = TIM_COUNTERMODE_UP;
    handle->Init.Period = 2499U;
    handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(handle) != HAL_OK) { Error_Handler(); }
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(handle, &oc, channel) != HAL_OK) { Error_Handler(); }
    if (second_channel != 0xFFFFFFFFUL &&
        HAL_TIM_PWM_ConfigChannel(handle, &oc, second_channel) != HAL_OK) { Error_Handler(); }
}

void MX_TIM2_Init(void) { configure(&htim2, TIM2, TIM_CHANNEL_2, 0xFFFFFFFFUL); }
void MX_TIM3_Init(void) { configure(&htim3, TIM3, TIM_CHANNEL_1, 0xFFFFFFFFUL); }
void MX_TIM4_Init(void) { configure(&htim4, TIM4, TIM_CHANNEL_3, TIM_CHANNEL_4); }

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *handle)
{
    if (handle->Instance == TIM2) { __HAL_RCC_TIM2_CLK_ENABLE(); }
    if (handle->Instance == TIM3) { __HAL_RCC_TIM3_CLK_ENABLE(); }
    if (handle->Instance == TIM4) { __HAL_RCC_TIM4_CLK_ENABLE(); }
}
