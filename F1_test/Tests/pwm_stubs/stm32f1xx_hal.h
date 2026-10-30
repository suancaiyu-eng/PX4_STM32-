#ifndef PWM_HOST_HAL_H
#define PWM_HOST_HAL_H
#include <stdint.h>
typedef struct {
    uint32_t CR1, CCER, CCR1, CCR2, CCR3, CCR4, CNT, EGR, SR, PSC, ARR, CCMR1, CCMR2;
} TIM_TypeDef;
typedef struct { uint32_t odr, mode[16], CRL, CRH; } GPIO_TypeDef;
typedef struct {
    uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
typedef struct { uint32_t Pin, Mode, Speed; } GPIO_InitTypeDef;
typedef struct { uint32_t APB1CLKDivider; } RCC_ClkInitTypeDef;
typedef struct { uint32_t MAPR; } AFIO_TypeDef;
extern AFIO_TypeDef host_afio;
#define AFIO (&host_afio)
#define AFIO_MAPR_TIM2_REMAP 0x300U
#define AFIO_MAPR_TIM3_REMAP 0xC00U
#define AFIO_MAPR_TIM4_REMAP 0x1000U
#define AFIO_MAPR_SWJ_CFG 0x7000000U
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE 0x2000000U
extern TIM_TypeDef host_tim2, host_tim3, host_tim4;
extern GPIO_TypeDef host_gpioa, host_gpiob;
extern uint32_t host_irq;
#define TIM2 (&host_tim2)
#define TIM3 (&host_tim3)
#define TIM4 (&host_tim4)
#define GPIOA (&host_gpioa)
#define GPIOB (&host_gpiob)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_RESET 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_SPEED_FREQ_HIGH 3U
#define TIM_CR1_CEN 1U
#define TIM_CR1_UDIS 2U
#define TIM_EGR_UG 1U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E (1U << 4)
#define TIM_CCER_CC3E (1U << 8)
#define TIM_CCER_CC4E (1U << 12)
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 0x80U
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define HAL_OK 0U
#define RCC_HCLK_DIV1 1U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM4_CLK_ENABLE() ((void)0)
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_AFIO_REMAP_TIM2_DISABLE() ((void)0)
#define __HAL_AFIO_REMAP_TIM3_DISABLE() ((void)0)
#define __HAL_AFIO_REMAP_TIM4_DISABLE() ((void)0)
static inline uint32_t __get_PRIMASK(void) { return host_irq; }
static inline void __disable_irq(void) { host_irq = 1U; }
static inline void __set_PRIMASK(uint32_t value) { host_irq = value; }
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, uint32_t);
void HAL_GPIO_Init(GPIO_TypeDef *, const GPIO_InitTypeDef *);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *, uint32_t *);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_TIM_PWM_Init(TIM_HandleTypeDef *);
uint32_t HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *, const TIM_OC_InitTypeDef *, uint32_t);
#endif
