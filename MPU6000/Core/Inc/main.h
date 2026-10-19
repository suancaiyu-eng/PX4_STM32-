/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSORS_EN_Pin GPIO_PIN_3
#define SENSORS_EN_GPIO_Port GPIOE
#define GYRO_CS_Pin GPIO_PIN_13
#define GYRO_CS_GPIO_Port GPIOC
#define ACCEL_MAG_CS_Pin GPIO_PIN_15
#define ACCEL_MAG_CS_GPIO_Port GPIOC
#define MPU_CS_Pin GPIO_PIN_2
#define MPU_CS_GPIO_Port GPIOC
#define MPU_DRDY_Pin GPIO_PIN_15
#define MPU_DRDY_GPIO_Port GPIOD
#define MPU_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define BARO_CS_Pin GPIO_PIN_7
#define BARO_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
