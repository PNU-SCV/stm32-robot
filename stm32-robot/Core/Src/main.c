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
#include "semphr.h"
#include "queue.h"

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
osThreadId_t lightControlTaskHandle;

osMessageQueueId_t proxQueueHandle;
osMessageQueueId_t cmdQueueHandle;
osTimerId_t watchdogTimerHandle;

SemaphoreHandle_t motorStatusSemaphore;

QueueHandle_t recvCmdQueue;

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t lightControlTask_attributes = {
  .name = "lightControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


const osMessageQueueAttr_t recvCmdQueue_attributes = {
  .name = "recvCmdQueue"
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
void MotorControlTask(void *argument);
void LightControlTask(void *argument);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void SendProximityDataToESP32(uint8_t status);
void ControlMotors(uint8_t cmd);
void ControlLights(uint8_t motor_status);
void ResetLights(void);

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

  motorStatusSemaphore = xSemaphoreCreateBinary();

  recvCmdQueue = xQueueCreate(10, sizeof(ESP32RecvData));

  watchdogTimerHandle = osTimerNew(WatchdogTimerCallback, osTimerOnce, NULL, &watchdogTimer_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  lightControlTaskHandle = osThreadNew(LightControlTask, NULL, &lightControlTask_attributes);
  motorControlTaskHandle = osThreadNew(MotorControlTask, NULL, &motorControlTask_attributes);

  osTimerStart(watchdogTimerHandle, 1000);
  xSemaphoreGive(motorStatusSemaphore);

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

  // 2. 왼쪽 빨간색 라이트 (GPIOB_PIN_8) 초기화 (Push-Pull Output)
      GPIO_InitStruct.Pin = LEFT_Light_Red_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-Pull Output 모드
      GPIO_InitStruct.Pull = GPIO_NOPULL;          // 내부 Pull-up/Pull-down 사용 안함
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 속도 설정 (LOW, MEDIUM, HIGH 가능)
      HAL_GPIO_Init(LEFT_Light_Red_Port, &GPIO_InitStruct);

      // 3. 왼쪽 초록색 라이트 (GPIOC_PIN_9) 초기화 (Push-Pull Output)
      GPIO_InitStruct.Pin = LEFT_Light_Green_Pin;
      HAL_GPIO_Init(LEFT_Light_Green_Port, &GPIO_InitStruct);

      // 4. 오른쪽 빨간색 라이트 (GPIOC_PIN_6) 초기화 (Push-Pull Output)
      GPIO_InitStruct.Pin = RIGHT_Light_Red_Pin;
      HAL_GPIO_Init(RIGHT_Light_Red_Port, &GPIO_InitStruct);

      // 5. 오른쪽 초록색 라이트 (GPIOC_PIN_8) 초기화 (Push-Pull Output)
      GPIO_InitStruct.Pin = RIGHT_Light_Green_Pin;
      HAL_GPIO_Init(RIGHT_Light_Green_Port, &GPIO_InitStruct);

      // 6. 초기 상태로 모든 핀을 RESET 상태로 설정 (OFF)
      HAL_GPIO_WritePin(LEFT_Light_Red_Port, LEFT_Light_Red_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LEFT_Light_Green_Port, LEFT_Light_Green_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RIGHT_Light_Red_Port, RIGHT_Light_Red_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RIGHT_Light_Green_Port, RIGHT_Light_Green_Pin, GPIO_PIN_RESET);

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
	ESP32RecvData esp32_cmd;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_buffer, sizeof(uart1_rx_buffer));
    esp32_cmd = uart1_rx_buffer;

    xQueueSendFromISR(recvCmdQueue, &esp32_cmd, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

void ControlLights(uint8_t motor_status)
{
    switch (motor_status)
    {
        case CMD_FORWARD:
            HAL_GPIO_TogglePin(LEFT_Light_Green_Port, LEFT_Light_Green_Pin);
            HAL_GPIO_TogglePin(RIGHT_Light_Green_Port, RIGHT_Light_Green_Pin);
            break;

        case CMD_COUNTERCLOCKWISE_ROTATE:
            HAL_GPIO_TogglePin(LEFT_Light_Red_Port, LEFT_Light_Red_Pin);
            HAL_GPIO_TogglePin(LEFT_Light_Green_Port, LEFT_Light_Green_Pin);
            break;

        case CMD_CLOCKWISE_ROTATE:
            HAL_GPIO_TogglePin(RIGHT_Light_Red_Port, RIGHT_Light_Red_Pin);
            HAL_GPIO_TogglePin(RIGHT_Light_Green_Port, RIGHT_Light_Green_Pin);
            break;

        case CMD_STOP:
            HAL_GPIO_WritePin(LEFT_Light_Red_Port, LEFT_Light_Red_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RIGHT_Light_Red_Port, RIGHT_Light_Red_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(LEFT_Light_Green_Port, LEFT_Light_Green_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RIGHT_Light_Green_Port, RIGHT_Light_Green_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}

void ResetLight(void)
{
	HAL_GPIO_WritePin(LEFT_Light_Red_Port, LEFT_Light_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEFT_Light_Green_Port, LEFT_Light_Green_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(RIGHT_Light_Red_Port, RIGHT_Light_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RIGHT_Light_Green_Port, RIGHT_Light_Green_Pin, GPIO_PIN_SET);
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

void MotorControlTask(void *argument)
{
	uint8_t motor_status = 0;
	ESP32RecvData recv_cmd;

	ControlMotors(CMD_FORWARD);

	for(;;)
	{
		if(proxFlag)
		{
			proxFlag = 0;

			SendProximityDataToESP32(proxStatus);
		}

		if(xQueueReceive(recvCmdQueue, &recv_cmd, portMAX_DELAY) == pdPASS)
		{
			osTimerStart(watchdogTimerHandle, 1000); // 1 second watchdog timer
			HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

			if(proxStatus == NO_OBSTACLE && xSemaphoreTake(motorStatusSemaphore, pdMS_TO_TICKS(10)) == pdFALSE) continue;

			motorStatus = recv_cmd.cmd;

			xSemaphoreGive(motorStatusSemaphore);

			motor_status = recv_cmd.cmd;

			ControlMotors(motor_status);
		}

		osDelay(10);
	}
}

void LightControlTask(void *argument)
{
	uint8_t cur_motor_status = 0, bef_motor_status = -1;

	for(;;)
	{
		if(xSemaphoreTake(motorStatusSemaphore, pdMS_TO_TICKS(10)) == pdFALSE) continue;

		cur_motor_status = motorStatus;

		xSemaphoreGive(motorStatusSemaphore);

		if(cur_motor_status != bef_motor_status) {
			bef_motor_status = cur_motor_status;
			ResetLight();
		}

		ControlLights(cur_motor_status);

		osDelay(500);
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
