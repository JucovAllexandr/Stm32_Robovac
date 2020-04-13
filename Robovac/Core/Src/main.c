/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "wheel_control.h"
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

volatile int rm_enc_count = 0;
volatile float freq = 0;
volatile float rpm = 0;
volatile uint8_t rm_enc_it = 0;
volatile uint64_t rm_enc_time_us = 0;
volatile uint64_t last_rm_enc_time_us = 0;
volatile uint64_t time_us = 0;
volatile WheelControl r_wheel = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void setGPIOMode(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t Mode);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	TIM2->CNT = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	wheel_init(&r_wheel, &huart1, RM_ENC_EN_GPIO_Port, RM_ENC_EN_Pin, &htim4,
			TIM_CHANNEL_2, TIM_CHANNEL_1, RM_1_1_GPIO_Port, RM_1_1_Pin,
			RM_2_1_Pin);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1152000;
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RM_2_1_Pin|RM_1_1_Pin|MB_S2_1_Pin|MB_S1_1_Pin 
                          |LM_1_1_Pin|LM_2_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MB_S1_2_Pin|LM_2_2_Pin|LM_1_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MB_S2_2_GPIO_Port, MB_S2_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RM_ENC_EN_GPIO_Port, RM_ENC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RM_2_1_Pin RM_1_1_Pin MB_S2_1_Pin MB_S1_1_Pin 
                           LM_1_1_Pin LM_2_1_Pin */
  GPIO_InitStruct.Pin = RM_2_1_Pin|RM_1_1_Pin|MB_S2_1_Pin|MB_S1_1_Pin 
                          |LM_1_1_Pin|LM_2_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RM_EXTI4_ENC_Pin */
  GPIO_InitStruct.Pin = RM_EXTI4_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RM_EXTI4_ENC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RM_SW_Pin */
  GPIO_InitStruct.Pin = RM_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RM_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MB_S1_2_Pin LM_2_2_Pin LM_1_2_Pin */
  GPIO_InitStruct.Pin = MB_S1_2_Pin|LM_2_2_Pin|LM_1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MB_S2_2_Pin */
  GPIO_InitStruct.Pin = MB_S2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MB_S2_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RM_ENC_EN_Pin */
  GPIO_InitStruct.Pin = RM_ENC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RM_ENC_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LM_SW_Pin */
  GPIO_InitStruct.Pin = LM_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LM_SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (RM_EXTI4_ENC_Pin == GPIO_Pin) {
		wheel_encoder_task(&r_wheel);

		/*if (time_us > 0) {
		 rm_enc_time_us = time_us - last_rm_enc_time_us;
		 //if (rm_enc_time_us < 100000) {
		 if (rm_enc_time_us > 0) {
		 freq = 1.0 / (rm_enc_time_us * 0.000001);
		 rpm = ((freq * 0.083) * 60.0);
		 if (rpm > 10000) {
		 rpm = 10000;
		 } else if (rpm < 0) {
		 rpm = 0;
		 }
		 //TIM4->CCR2 = pid_calc(pid, rpm, 4000);
		 }

		 //}
		 }*/

		//rm_enc_it++;
		//last_rm_enc_time_us = time_us;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		time_us += 50;
		r_wheel.time_us = time_us;
	}
}

void setGPIOMode(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t Mode) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	turn_clockwise(&r_wheel, 4000);
	/* Infinite loop */
	//vTaskDelay(10000 / portTICK_RATE_MS);
	//turn_counter_clockwise(r_wheel, 0);
	//vTaskDelay(10000 / portTICK_RATE_MS);
	//turn_counter_clockwise(r_wheel, 9000);
	//vTaskDelay(10000 / portTICK_RATE_MS);
	//turn_counter_clockwise(r_wheel, 0);
	//TIM4->CCR1 = 100;
	//TIM4->CCR2 = 100;
	//HAL_GPIO_WritePin(RM_ENC_EN_GPIO_Port, RM_ENC_EN_Pin, GPIO_PIN_SET);
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	/*setGPIOMode(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_MODE_OUTPUT_PP);

	 HAL_GPIO_WritePin(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_PIN_RESET);

	 setGPIOMode(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_MODE_INPUT);

	 HAL_GPIO_WritePin(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_PIN_SET);*/

	/*setGPIOMode(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_MODE_INPUT);

	 HAL_GPIO_WritePin(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_PIN_SET);

	 setGPIOMode(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_MODE_OUTPUT_PP);

	 HAL_GPIO_WritePin(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_PIN_RESET);*/
	char buffer[30] = { 0 };
	uint32_t compare = 0;

	for (;;) {

		if (r_wheel.status == TurnCkw) {
			compare = __HAL_TIM_GET_COMPARE(r_wheel.htim, r_wheel.channel_ckw);
		} else if (r_wheel.status == TurnC_ckw) {
			compare = __HAL_TIM_GET_COMPARE(r_wheel.htim, r_wheel.channel_c_ckw);
		}

		sprintf(buffer, "!%d %d\r\n", (int)r_wheel.current_rpm, compare);

		//if (ret >= sizeof buffer || ret >= (sizeof buffer / 2)) {
		HAL_UART_Transmit(r_wheel.huart, buffer, sizeof buffer, 100);
		//}

		osDelay(1);
		/*if (rm_enc_count >= 2)
		 {
		 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		 setGPIOMode(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_MODE_OUTPUT_PP);
		 HAL_GPIO_WritePin(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_PIN_RESET);

		 }*/
		//osDelay(1);
		/*if (t >= 1) {
		 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
		 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

		 setGPIOMode(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_MODE_OUTPUT_PP);
		 //setGPIOMode(RM_2_2_GPIO_Port, RM_2_2_Pin, GPIO_MODE_OUTPUT_PP);

		 //HAL_GPIO_WritePin(RM_1_2_GPIO_Port, RM_1_2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_PIN_RESET);

		 //setGPIOMode(RM_1_2_GPIO_Port, RM_1_2_Pin, GPIO_MODE_INPUT);
		 setGPIOMode(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_MODE_INPUT);

		 HAL_GPIO_WritePin(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_PIN_SET);
		 //HAL_GPIO_WritePin(RM_2_2_GPIO_Port, RM_2_2_Pin, GPIO_PIN_SET);
		 t=0;
		 vTaskDelay(5000 / portTICK_RATE_MS);

		 } else if(t==0){

		 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		 //setGPIOMode(RM_1_2_GPIO_Port, RM_1_2_Pin, GPIO_MODE_OUTPUT_PP);
		 setGPIOMode(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_MODE_OUTPUT_PP);

		 HAL_GPIO_WritePin(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_PIN_RESET);
		 //HAL_GPIO_WritePin(RM_2_2_GPIO_Port, RM_2_2_Pin, GPIO_PIN_RESET);

		 setGPIOMode(RM_1_1_GPIO_Port, RM_1_1_Pin, GPIO_MODE_INPUT);
		 //setGPIOMode(RM_2_2_GPIO_Port, RM_2_2_Pin, GPIO_MODE_INPUT);

		 //HAL_GPIO_WritePin(RM_1_2_GPIO_Port, RM_1_2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(RM_2_1_GPIO_Port, RM_2_1_Pin, GPIO_PIN_SET);
		 t=1;
		 vTaskDelay(5000 / portTICK_RATE_MS);
		 }*/

		/*setGPIOMode(LM_1_1_GPIO_Port, LM_1_1_Pin, GPIO_MODE_OUTPUT_PP);
		 setGPIOMode(LM_2_2_GPIO_Port, LM_2_2_Pin, GPIO_MODE_OUTPUT_PP);

		 HAL_GPIO_WritePin(LM_1_2_GPIO_Port, LM_1_2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(LM_2_1_GPIO_Port, LM_2_1_Pin, GPIO_PIN_RESET);

		 setGPIOMode(LM_1_2_GPIO_Port, LM_1_2_Pin, GPIO_MODE_INPUT);
		 setGPIOMode(LM_2_1_GPIO_Port, LM_2_1_Pin, GPIO_MODE_INPUT);

		 HAL_GPIO_WritePin(LM_1_1_GPIO_Port, LM_1_1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(LM_2_2_GPIO_Port, LM_2_2_Pin, GPIO_PIN_SET);*/

		/*setGPIOMode(LM_1_2_GPIO_Port, LM_1_2_Pin, GPIO_MODE_OUTPUT_PP);
		 setGPIOMode(LM_2_1_GPIO_Port, LM_2_1_Pin, GPIO_MODE_OUTPUT_PP);

		 HAL_GPIO_WritePin(LM_1_1_GPIO_Port, LM_1_1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(LM_2_2_GPIO_Port, LM_2_2_Pin, GPIO_PIN_RESET);

		 setGPIOMode(LM_1_1_GPIO_Port, LM_1_1_Pin, GPIO_MODE_INPUT);
		 setGPIOMode(LM_2_2_GPIO_Port, LM_2_2_Pin, GPIO_MODE_INPUT);

		 HAL_GPIO_WritePin(LM_1_2_GPIO_Port, LM_1_2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(LM_2_1_GPIO_Port, LM_2_1_Pin, GPIO_PIN_SET);*/

		/*setGPIOMode(mb_s1_1_GPIO_Port, mb_s1_1_Pin, GPIO_MODE_OUTPUT_PP);
		 setGPIOMode(mb_s2_2_GPIO_Port, mb_s2_2_Pin, GPIO_MODE_OUTPUT_PP);

		 HAL_GPIO_WritePin(mb_s1_2_GPIO_Port, mb_s1_2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(mb_s2_1_GPIO_Port, mb_s2_1_Pin, GPIO_PIN_RESET);

		 setGPIOMode(mb_s1_2_GPIO_Port, mb_s1_2_Pin, GPIO_MODE_INPUT);
		 setGPIOMode(mb_s2_1_GPIO_Port, mb_s2_1_Pin, GPIO_MODE_INPUT);

		 HAL_GPIO_WritePin(mb_s1_1_GPIO_Port, mb_s1_1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(mb_s2_2_GPIO_Port, mb_s2_2_Pin, GPIO_PIN_SET);*/

		/*osDelay(2000);

		 setGPIOMode(mb_s1_2_GPIO_Port, mb_s1_2_Pin, GPIO_MODE_OUTPUT_PP);
		 setGPIOMode(mb_s2_1_GPIO_Port, mb_s2_1_Pin, GPIO_MODE_OUTPUT_PP);

		 HAL_GPIO_WritePin(mb_s1_1_GPIO_Port, mb_s1_1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(mb_s2_2_GPIO_Port, mb_s2_2_Pin, GPIO_PIN_RESET);

		 setGPIOMode(mb_s1_1_GPIO_Port, mb_s1_1_Pin, GPIO_MODE_INPUT);
		 setGPIOMode(mb_s2_2_GPIO_Port, mb_s2_2_Pin, GPIO_MODE_INPUT);

		 HAL_GPIO_WritePin(mb_s1_2_GPIO_Port, mb_s1_2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(mb_s2_1_GPIO_Port, mb_s2_1_Pin, GPIO_PIN_SET);*/

	}
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
