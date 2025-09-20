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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM20948.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ICM20948 imu;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


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
#define PWM_B_IN1_Pin GPIO_PIN_5
#define PWM_B_IN1_GPIO_Port GPIOE
#define PWM_B_IN2_Pin GPIO_PIN_6
#define PWM_B_IN2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define PWM_SERVO_Pin GPIO_PIN_14
#define PWM_SERVO_GPIO_Port GPIOB
#define SN_US_TRIG_Pin GPIO_PIN_15
#define SN_US_TRIG_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RST_Pin GPIO_PIN_12
#define OLED_RST_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOD
#define SN_US_ECHO_Pin GPIO_PIN_7
#define SN_US_ECHO_GPIO_Port GPIOC
#define PWM_A_IN2_Pin GPIO_PIN_8
#define PWM_A_IN2_GPIO_Port GPIOB
#define PWM_A_IN1_Pin GPIO_PIN_9
#define PWM_A_IN1_GPIO_Port GPIOB
#define USER_PB_Pin GPIO_PIN_0
#define USER_PB_GPIO_Port GPIOE
#define USER_PB_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
