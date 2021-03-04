/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
UART_HandleTypeDef huart3;
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
//void tilt_callback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_VBAT_Pin GPIO_PIN_0
#define PWR_VBAT_GPIO_Port GPIOA
#define PWR_VMON_Pin GPIO_PIN_1
#define PWR_VMON_GPIO_Port GPIOA
#define PWR_EN_Pin GPIO_PIN_0
#define PWR_EN_GPIO_Port GPIOB
#define RSTNB_Pin GPIO_PIN_1
#define RSTNB_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define HARD_RST_Pin GPIO_PIN_8
#define HARD_RST_GPIO_Port GPIOA
#define LSM6DSL_INT1_Pin GPIO_PIN_5
#define LSM6DSL_INT1_GPIO_Port GPIOB
#define LSM6DSL_INT1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
#define TX_BUF_DIM          1000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
