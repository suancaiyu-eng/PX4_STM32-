#include "usart.h"

UART_HandleTypeDef huart3;

void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 100000U;
    /* STM32 word length includes parity: 9B + parity means eight payload bits. */
    huart3.Init.WordLength = UART_WORDLENGTH_9B;
    huart3.Init.StopBits = UART_STOPBITS_2;
    huart3.Init.Parity = UART_PARITY_EVEN;
    huart3.Init.Mode = UART_MODE_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* RC IN is already inverted by U903. PB10/S.BUS OUT is not enabled. */
        gpio.Pin = GPIO_PIN_11;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_NVIC_SetPriority(USART3_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
    }
}
