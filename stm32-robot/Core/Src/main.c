/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId_t defaultTaskHandle;
osThreadId_t motorControlTaskHandle;
osMessageQueueId_t proxQueueHandle;
osMessageQueueId_t cmdQueueHandle;
osTimerId_t watchdogTimerHandle;

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


const osMessageQueueAttr_t cmdQueue_attributes = {
  .name = "cmdQueue"
};

const osTimerAttr_t watchdogTimer_attributes = {
  .name = "watchdogTimer"
};

/* USER CODE BEGIN PV */
volatile uint8_t proxStatus = NO_OBSTACLE;
volatile uint8_t motorStatus = CMD_STOP;
ESP32RecvData uart1_rx_buffer;

volatile uint8_t proxFlag = 0;
volatile uint16_t proxGPIO_Pin;
volatile uint8_t uartFlag = 0;
volatile ESP32RecvData uartCmd;
volatile uint8_t obstacle_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void SendProximityDataToESP32(uint8_t status);
void ControlMotors(uint8_t cmd);
void WatchdogTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();

  osKernelInitialize();

  cmdQueueHandle = osMessageQueueNew(10, sizeof(ESP32RecvData), &cmdQueue_attributes);
  watchdogTimerHandle = osTimerNew(WatchdogTimerCallback, osTimerOnce, NULL, &watchdogTimer_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  motorControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &motorControlTask_attributes);

  osTimerStart(watchdogTimerHandle, 1000);

  HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_buffer, sizeof(uart1_rx_buffer));

  osKernelStart();

  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Forward_Pin|LEFT_Motor_Backward_Pin|RIGHT_Motor_Forward_Pin|RIGHT_Motor_Backward_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin|LEFT_Prox_Pin|MID_Prox_Pin|RIGHT_Prox_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_Motor_Forward_Pin|LEFT_Motor_Backward_Pin|RIGHT_Motor_Forward_Pin|RIGHT_Motor_Backward_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  GPIO_InitStruct.Pin = USART1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USART1_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USART1_RX_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == LEFT_Prox_Pin || GPIO_Pin == MID_Prox_Pin || GPIO_Pin == RIGHT_Prox_Pin)
  {
	  proxFlag = 1;

	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_SET) {
		  proxStatus = 0;
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  } else {
		  proxStatus = GPIO_Pin + 1;
		  ControlMotors(CMD_STOP);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_buffer, sizeof(uart1_rx_buffer));
    uartFlag = 1;
    uartCmd = uart1_rx_buffer;
  }
}

void SendProximityDataToESP32(uint8_t status)
{
  ESP32SendData data;
  data.status = status;
  HAL_UART_Transmit(&huart1, (uint8_t*)&data, sizeof(data), HAL_MAX_DELAY);
}

void ControlMotors(uint8_t cmd)
{
  switch(cmd)
  {
    case CMD_STOP:
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Forward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Backward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Forward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Backward_Pin, GPIO_PIN_RESET);
      break;
    case CMD_FORWARD:
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Forward_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Backward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Forward_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Backward_Pin, GPIO_PIN_RESET);
      break;
    case CMD_CLOCKWISE_ROTATE:
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Forward_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Backward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Forward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Backward_Pin, GPIO_PIN_SET);
      break;
    case CMD_COUNTERCLOCKWISE_ROTATE:
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Forward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Backward_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Forward_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Backward_Pin, GPIO_PIN_RESET);
      break;
    default:
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Forward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LEFT_Motor_Backward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Forward_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RIGHT_Motor_Backward_Pin, GPIO_PIN_RESET);
      break;
  }
}

void WatchdogTimerCallback(void *argument)
{
  ControlMotors(CMD_STOP);
}

void StartDefaultTask(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void StartMotorControlTask(void *argument)
{
	ESP32RecvData recvData;

	ControlMotors(CMD_FORWARD);

	for(;;)
	{
		if(proxFlag)
		{
			proxFlag = 0;

			SendProximityDataToESP32(proxStatus);
		}

		if(uartFlag)
		{
			uartFlag = 0;
			recvData.cmd = uartCmd.cmd;

			osTimerStart(watchdogTimerHandle, 1000); // 1 second watchdog timer

			if(proxStatus == NO_OBSTACLE)
				ControlMotors(recvData.cmd);
		}

		osDelay(10);
	}
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
