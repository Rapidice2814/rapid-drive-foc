/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define ADC1_CHANNELS 4
#define PWM_CLOCK_DIVIDER 3542
#define DEBUG_LOOP_CLOCK_DIVIDER 200
#define PWM_FREQUENCY (CLOCK_FREQUENCY/(PWM_CLOCK_DIVIDER*2))
#define CLOCK_FREQUENCY 170000000
#define ENCODER_PULSES_PER_ROTATION 4096
#define CURRENT_LOOP_FREQUENCY (PWM_FREQUENCY/CURRENT_LOOP_CLOCK_DIVIDER)
#define ADC2_CHANNELS 1
#define DEBUG_FREQUENCY (PWM_FREQUENCY/DEBUG_LOOP_CLOCK_DIVIDER)
#define CURRENT_LOOP_CLOCK_DIVIDER 3
#define DRV_ENABLE_Pin GPIO_PIN_13
#define DRV_ENABLE_GPIO_Port GPIOC
#define DRV_NCS_Pin GPIO_PIN_14
#define DRV_NCS_GPIO_Port GPIOC
#define DRV_NFAULT_Pin GPIO_PIN_15
#define DRV_NFAULT_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_10
#define Button_GPIO_Port GPIOG
#define CURRENT_A_Pin GPIO_PIN_0
#define CURRENT_A_GPIO_Port GPIOA
#define CURRENT_B_Pin GPIO_PIN_1
#define CURRENT_B_GPIO_Port GPIOA
#define CURRENT_C_Pin GPIO_PIN_2
#define CURRENT_C_GPIO_Port GPIOA
#define V_BAT_Pin GPIO_PIN_3
#define V_BAT_GPIO_Port GPIOA
#define Thermistor_Pin GPIO_PIN_4
#define Thermistor_GPIO_Port GPIOA
#define DEBUG_LED0_Pin GPIO_PIN_5
#define DEBUG_LED0_GPIO_Port GPIOA
#define DEBUG_LED1_Pin GPIO_PIN_6
#define DEBUG_LED1_GPIO_Port GPIOA
#define DEBUG_LED2_Pin GPIO_PIN_7
#define DEBUG_LED2_GPIO_Port GPIOA
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define AS_NCS_Pin GPIO_PIN_12
#define AS_NCS_GPIO_Port GPIOB
#define INL_ALL_Pin GPIO_PIN_6
#define INL_ALL_GPIO_Port GPIOC
#define PWM_C_Pin GPIO_PIN_8
#define PWM_C_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_10
#define PWM_A_GPIO_Port GPIOA
#define ENC_I_Pin GPIO_PIN_3
#define ENC_I_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_4
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_5
#define ENC_B_GPIO_Port GPIOB
#define RGB_Timer_Pin GPIO_PIN_6
#define RGB_Timer_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
