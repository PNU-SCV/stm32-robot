/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * (C)opyright 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

#define ESP32_UART_BAUD 115200

#define CMD_STOP (uint8_t)0x00
#define CMD_FORWARD (uint8_t)0x01
#define CMD_CLOCKWISE_ROTATE (uint8_t)0x02
#define CMD_COUNTERCLOCKWISE_ROTATE (uint8_t)0x03

#define NO_OBSTACLE (uint8_t)0x00
#define LEFT_OBSTACLE (uint8_t)0x01
#define MID_OBSTACLE (uint8_t)0x02
#define RIGHT_OBSTACLE (uint8_t)0x03

typedef struct {
    uint8_t status;
} ESP32SendData;

typedef struct {
    uint8_t cmd;
} ESP32RecvData;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA

#define LEFT_Motor_Forward_Pin GPIO_PIN_11
#define LEFT_Motor_Backward_Pin GPIO_PIN_12
#define RIGHT_Motor_Forward_Pin GPIO_PIN_13
#define RIGHT_Motor_Backward_Pin GPIO_PIN_14
#define LEFT_Motor_GPIO_Port GPIOB
#define RIGHT_Motor_GPIO_Port GPIOB

#define LEFT_Prox_Pin GPIO_PIN_0
#define LEFT_Prox_GPIO_Port GPIOC
#define MID_Prox_Pin GPIO_PIN_1
#define MID_Prox_GPIO_Port GPIOC
#define RIGHT_Prox_Pin GPIO_PIN_2
#define RIGHT_Prox_GPIO_Port GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
