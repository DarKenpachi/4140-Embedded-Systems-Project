/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*              ultrasonic sensor 1           */
volatile uint32_t pulse_width = 0;
volatile uint32_t signal_polarity = 0;
volatile uint32_t last_captured = 0;
volatile uint32_t distanceCM = 0;

/*              ultrasonic sensor 2           */
volatile uint32_t pulse_width2 = 0;
volatile uint32_t signal_polarity2 = 0;
volatile uint32_t last_captured2 = 0;
volatile uint32_t distanceCM2 = 0;

/*              ultrasonic sensor 3           */
volatile uint32_t pulse_width3 = 0;
volatile uint32_t signal_polarity3 = 0;
volatile uint32_t last_captured3 = 0;
volatile uint32_t distanceCM3 = 0;

/***************************************************************************/
/*                         For the blue button on the board               */
/**************************************************************************/
GPIO_PinState button_state; // external pushbutton (PA4)


/***************************************************************************/
/*                         RGB LED module                                 */
/**************************************************************************/
GPIO_PinState Red;
GPIO_PinState Green;
GPIO_PinState Blue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void delayMicroseconds(uint32_t microseconds);
void Trigger_Pulse();
void Trigger_Pulse2();
void Trigger_Pulse3();
void Buzzer_Off();
void Buzzer_Tone(uint32_t frequency);
void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue);
void RGB_Off();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); //start the timer in input capture interrupt mode
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2); //start the timer in input capture interrupt mode
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3); //start the timer in input capture interrupt mode
  HAL_TIM_Base_Start_IT(&htim4); //start base interrupt for overflow tracking

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  pulse_width = 0;
  signal_polarity = 0;
  last_captured = 0;

  pulse_width2 = 0;
  signal_polarity2 = 0;
  last_captured2 = 0;

  pulse_width3 = 0;
  signal_polarity3 = 0;
  last_captured3 = 0;

  uint8_t button_prev_state = GPIO_PIN_SET; // assume button not pressed initially
  uint8_t button_flag = 0;					// Flag: 0 = stopped, 1 = running

  // Initialize RGB LED states
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);   // Red
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  // Green
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  // Blue



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // --- Read button state ---
	 button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

	 // --- Detect rising edge of button press ---
	 if (button_prev_state == GPIO_PIN_SET && button_state == GPIO_PIN_RESET)
	 {
		 button_flag = 1;  // start measurement on first press
	 }
	 /* Save current state for next loop */
	 button_prev_state = button_state;

	// ------- call the triggers ----------
	 if (button_flag)
	 {
	     Trigger_Pulse();
	     //Trigger_Pulse2();
	     //Trigger_Pulse3();
	     HAL_Delay(50);

	     if (distanceCM < 5)
	     {
	         // Object very close → constant tone
	         Buzzer_Tone(2000);
	         RGB_SetColor(1,0,0);
	     }
	     else if (distanceCM < 15)
	     {
	         // Fast beeping
	         Buzzer_Tone(2000);
	         RGB_SetColor(1,0,0);
	         HAL_Delay(50);      // short beep
	         Buzzer_Off();
	         RGB_Off();
	         HAL_Delay(50);
	     }
	     else if (distanceCM < 30)
	     {
	         // Medium-speed beeping
	         Buzzer_Tone(2000);
	         RGB_SetColor(1,0,0);
	         HAL_Delay(100);
	         Buzzer_Off();
	         RGB_Off();
	         HAL_Delay(150);
	     }
	     else if (distanceCM < 50)
	     {
	         // Slow beeping
	         Buzzer_Tone(2000);
	         RGB_SetColor(0,1,0);
	         HAL_Delay(150);
	         Buzzer_Off();
	         RGB_Off();
	         HAL_Delay(350);
	     }
	     else
	     {
	         // Too far → silent
	         Buzzer_Off();
	         RGB_Off();
	     }
	 }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Red_Pin|Trigger_Pin|Trigger2_Pin|Trigger3_Pin
                          |Blue_Pin|Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : External_Pushbutton_Pin */
  GPIO_InitStruct.Pin = External_Pushbutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(External_Pushbutton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Red_Pin Trigger_Pin Trigger2_Pin Trigger3_Pin
                           Blue_Pin Green_Pin */
  GPIO_InitStruct.Pin = Red_Pin|Trigger_Pin|Trigger2_Pin|Trigger3_Pin
                          |Blue_Pin|Green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delayMicroseconds(uint32_t microseconds)
{
    // NOTE: This assumes SystemCoreClock holds the correct clock frequency (e.g., 4000000 Hz)
    // You may need to replace SystemCoreClock with the value of the clock used by the SysTick timer if they differ.
    uint32_t current_ticks = SysTick->LOAD + 1; // Number of ticks per second (usually SystemCoreClock)
    uint32_t ticks_per_us = current_ticks / 1000000;

    // Calculate the target number of ticks to wait for
    uint32_t ticks_to_wait = microseconds * ticks_per_us;

    // Get the current value of the SysTick counter register
    uint32_t start_tick = SysTick->VAL;

    // Loop until the required number of ticks has elapsed.
    // This is a simple, non-HAL busy-wait for short, critical timing.
    while ((SysTick->VAL - start_tick) < ticks_to_wait)
    {
        // Busy-wait
    }
}

void Trigger_Pulse()
{
    GPIOB -> ODR |= (1 << 13); //set PB13 high
    delayMicroseconds(10); //10 micro second pulse
    GPIOB -> ODR &= ~(1 << 13); // set PB13 low. clear pin
}

void Trigger_Pulse2()
{
    GPIOB -> ODR |= (1 << 14); //set PB14 high
    delayMicroseconds(10); //10 micro second pulse
    GPIOB -> ODR &= ~(1 << 14); // set PB14 low. clear pin
}

void Trigger_Pulse3()
{
    GPIOB -> ODR |= (1 << 15); //set PB15 high
    delayMicroseconds(10); //10 micro second pulse
    GPIOB -> ODR &= ~(1 << 15); // set PB15 low. clear pin
}

// Simple tone generator for passive buzzer
void Buzzer_Tone(uint32_t frequency)
{
	if(frequency == 0){
		Buzzer_Off();
		return;
	}

    uint32_t timer_clock = 1000000; // from Pre-scaler
    uint32_t period = timer_clock / frequency - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim3, period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period / 2);  // 50% duty
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

// Stop buzzer sound
void Buzzer_Off(void)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void RGB_Off(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Red
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  // Green
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  // Blue
}

void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
