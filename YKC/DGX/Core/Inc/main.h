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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Watchdog_INPUT_Pin GPIO_PIN_3
#define Watchdog_INPUT_GPIO_Port GPIOE
#define Watchdog_OUTPUT_Pin GPIO_PIN_4
#define Watchdog_OUTPUT_GPIO_Port GPIOE
#define SPI_UART_RX_LED_Pin GPIO_PIN_10
#define SPI_UART_RX_LED_GPIO_Port GPIOF
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define SPI_UART_TX_LED_Pin GPIO_PIN_11
#define SPI_UART_TX_LED_GPIO_Port GPIOF
#define RX_1_RX_LED_Pin GPIO_PIN_0
#define RX_1_RX_LED_GPIO_Port GPIOG
#define RX_1_TX_LED_Pin GPIO_PIN_1
#define RX_1_TX_LED_GPIO_Port GPIOG
#define RX_2_RX_LED_Pin GPIO_PIN_2
#define RX_2_RX_LED_GPIO_Port GPIOG
#define RX_2_TX_LED_Pin GPIO_PIN_3
#define RX_2_TX_LED_GPIO_Port GPIOG
#define RX_3_RX_LED_Pin GPIO_PIN_4
#define RX_3_RX_LED_GPIO_Port GPIOG
#define RX_3_TX_LED_Pin GPIO_PIN_5
#define RX_3_TX_LED_GPIO_Port GPIOG
#define RX_4_RX_LED_Pin GPIO_PIN_6
#define RX_4_RX_LED_GPIO_Port GPIOG
#define RX_4_TX_LED_Pin GPIO_PIN_7
#define RX_4_TX_LED_GPIO_Port GPIOG
#define RX_5_RX_LED_Pin GPIO_PIN_8
#define RX_5_RX_LED_GPIO_Port GPIOG
#define SPI_IRQ_Pin GPIO_PIN_8
#define SPI_IRQ_GPIO_Port GPIOA
#define SPI_RST_Pin GPIO_PIN_11
#define SPI_RST_GPIO_Port GPIOA
#define RX_5_TX_LED_Pin GPIO_PIN_9
#define RX_5_TX_LED_GPIO_Port GPIOG
#define RX_6_RX_LED_Pin GPIO_PIN_10
#define RX_6_RX_LED_GPIO_Port GPIOG
#define RX_6_TX_LED_Pin GPIO_PIN_11
#define RX_6_TX_LED_GPIO_Port GPIOG
#define RX_7_RX_LED_Pin GPIO_PIN_12
#define RX_7_RX_LED_GPIO_Port GPIOG
#define RX_7_TX_LED_Pin GPIO_PIN_13
#define RX_7_TX_LED_GPIO_Port GPIOG
#define RX_8_RX_LED_Pin GPIO_PIN_14
#define RX_8_RX_LED_GPIO_Port GPIOG
#define RX_8_TX_LED_Pin GPIO_PIN_15
#define RX_8_TX_LED_GPIO_Port GPIOG
#define MCU_Status_1_Pin GPIO_PIN_6
#define MCU_Status_1_GPIO_Port GPIOB
#define MCU_Status_2_Pin GPIO_PIN_7
#define MCU_Status_2_GPIO_Port GPIOB
#define MCU_Status_3_Pin GPIO_PIN_8
#define MCU_Status_3_GPIO_Port GPIOB
#define MCU_Status_4_Pin GPIO_PIN_9
#define MCU_Status_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
