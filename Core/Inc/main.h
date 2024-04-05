/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STM32_BlueBtn_Pin GPIO_PIN_13
#define STM32_BlueBtn_GPIO_Port GPIOC
#define IR_8_Pin GPIO_PIN_0
#define IR_8_GPIO_Port GPIOC
#define IR_1_Pin GPIO_PIN_0
#define IR_1_GPIO_Port GPIOA
#define IR_2_Pin GPIO_PIN_1
#define IR_2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define IR_3_Pin GPIO_PIN_4
#define IR_3_GPIO_Port GPIOA
#define IR_4_Pin GPIO_PIN_6
#define IR_4_GPIO_Port GPIOA
#define IR_5_Pin GPIO_PIN_7
#define IR_5_GPIO_Port GPIOA
#define IR_6_Pin GPIO_PIN_0
#define IR_6_GPIO_Port GPIOB
#define IR_7_Pin GPIO_PIN_1
#define IR_7_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_6
#define AIN2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_7
#define AIN1_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_8
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_9
#define BIN2_GPIO_Port GPIOC
#define PWM_1_Pin GPIO_PIN_8
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_9
#define PWM_2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_11
#define SW2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
