/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
void APP_IWDG_Refresh(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PRIMARY_SLAVE_ADDR 0x59
#define ENUM_SLAVE_ADDR 0x7F
#define USART4_TX_Pin GPIO_PIN_0
#define USART4_TX_GPIO_Port GPIOA
#define USART4_RX_Pin GPIO_PIN_1
#define USART4_RX_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define CTRL3V3_nPGOOD_Pin GPIO_PIN_4
#define CTRL3V3_nPGOOD_GPIO_Port GPIOA
#define GPS_nEN_Pin GPIO_PIN_5
#define GPS_nEN_GPIO_Port GPIOA
#define MAG_INT_DRDY_Pin GPIO_PIN_0
#define MAG_INT_DRDY_GPIO_Port GPIOB
#define MAG_INT_DRDY_EXTI_IRQn EXTI0_1_IRQn
#define ACCEL_MAG_INT1_DRDY_Pin GPIO_PIN_1
#define ACCEL_MAG_INT1_DRDY_GPIO_Port GPIOB
#define ACCEL_MAG_INT1_DRDY_EXTI_IRQn EXTI0_1_IRQn
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA
#define I2C1_SCL_Pin GPIO_PIN_9
#define I2C1_SCL_GPIO_Port GPIOA
#define I2C1_SDA_Pin GPIO_PIN_10
#define I2C1_SDA_GPIO_Port GPIOA
#define ERROR_Pin GPIO_PIN_11
#define ERROR_GPIO_Port GPIOA
#define nTEST_Pin GPIO_PIN_12
#define nTEST_GPIO_Port GPIOA
#define STM_SWDIO_Pin GPIO_PIN_13
#define STM_SWDIO_GPIO_Port GPIOA
#define STM_SWCLK_Pin GPIO_PIN_14
#define STM_SWCLK_GPIO_Port GPIOA
#define I2C3_SDA_Pin GPIO_PIN_4
#define I2C3_SDA_GPIO_Port GPIOB
#define INT_OUT_Pin GPIO_PIN_5
#define INT_OUT_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LOG_UART            USART2
#define GPS_UART            USART1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
