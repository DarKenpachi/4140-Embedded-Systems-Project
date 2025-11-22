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
#include <math.h>
#include "motor_control.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NORM_SPEED 800
#define LOW_SPEED 600
#define SAFE_DIST 80.0f
#define CRICT_DIST 50.0f
#define DANGER_DIST 20.0f


#define Red_Pin GPIO_PIN_8
#define Red_Port GPIOA

#define Green_Pin GPIO_PIN_5
#define Green_Port GPIOB

#define Blue_Pin GPIO_PIN_5
#define Blue_Port GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

/* USER CODE BEGIN PV */

/*              ultrasonic sensor 1 (front)          */
volatile uint32_t pulse_width = 0;
volatile uint32_t signal_polarity = 0;
volatile uint32_t last_captured = 0;
volatile float distanceCM = 0;

/*              ultrasonic sensor 2 (left)         */
volatile uint32_t pulse_width2 = 0;
volatile uint32_t signal_polarity2 = 0;
volatile uint32_t last_captured2 = 0;
volatile float distanceCM2 = 0;

/*              ultrasonic sensor 3 (right)         */
volatile uint32_t pulse_width3 = 0;
volatile uint32_t signal_polarity3 = 0;
volatile uint32_t last_captured3 = 0;
volatile float distanceCM3 = 0;

/*              ultrasonic sensor 4 (rear)         */
volatile uint32_t pulse_width4 = 0;
volatile uint32_t signal_polarity4 = 0;
volatile uint32_t last_captured4 = 0;
volatile float distanceCM4 = 0;

char buf[16]; // for OLED.

/***************************************************************************/
/*                         For push button to start                       */
/**************************************************************************/
GPIO_PinState button_state; //PC13

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
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Trigger_Pulse();
void delayMicroseconds(uint32_t);
void Buzzer_Off();
void Buzzer_Tone(uint32_t frequency);
void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue);
void RGB_Off();
void BuzzerLED(float distance_cm);
void oled_Display(float dist_cm);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  SSD1306_Init (); // initialise the display
  Motor_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint8_t button_prev_state = GPIO_PIN_SET; // assume button not pressed initially
  static uint8_t button_flag = 0;					// Flag: 0 = stopped, 1 = running


  while (1)
  {
	// --- Read button state ---
    button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    // --- Detect rising edge of button press ---
	 if (button_prev_state == GPIO_PIN_SET && button_state == GPIO_PIN_RESET)
	 {
		button_flag = 1;
	 }
	 /* Save current state for next loop */
	 button_prev_state = button_state;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 if (button_flag)
	 {
		 // Trigger sensors and wait for new distance readings //
		 Trigger_Pulse();
		 HAL_Delay(50); // wait for echoes to update

		 //------------------- Safety Check for faulty sensor ------------------------//
		 /*
		 if (distanceCM > 450.0f)
		 {
			 distanceCM = distanceCM / 100.0f;
		 }
		 if (distanceCM2 > 450.0f)
		 {
			 distanceCM2 = distanceCM2 / 100.0f;
		 }
		 if (distanceCM3 > 450.0f)
		 {
			 distanceCM3 = distanceCM3 / 100.0f;
		 }
		 */
		 //----------------------------------------------------------------------------//

		 // show distance on screen and buzzer feedback //
		 BuzzerLED(distanceCM);
		 oled_Display(distanceCM);

		 if (distanceCM < SAFE_DIST)
		 {
		     TIM2->CCR1 = LOW_SPEED;
		     TIM2->CCR2 = LOW_SPEED;

		     // Obstacle ahead — decide direction
		     if (distanceCM2 > distanceCM3)
		     {
		         // More space on left → turn left
		         Motor_Direction(MOTOR_BACKWARD, MOTOR_FORWARD);
		         TIM2->CCR1 = LOW_SPEED;
		         TIM2->CCR2 = LOW_SPEED;
		     }
		     else if (distanceCM3 > distanceCM2)
		     {
		         // More space on right → turn right
		         Motor_Direction(MOTOR_FORWARD, MOTOR_BACKWARD);
		         TIM2->CCR1 = LOW_SPEED;
		         TIM2->CCR2 = LOW_SPEED;
		     }
		     else if (distanceCM3 < CRICT_DIST && distanceCM2 < CRICT_DIST)
		     {
		    	    // reverse
		    	    while ((distanceCM3 < SAFE_DIST && distanceCM2 < SAFE_DIST && distanceCM < SAFE_DIST) || distanceCM4 > 100.0f)
		    	    {
		    	        Motor_Direction(MOTOR_BACKWARD, MOTOR_BACKWARD);

		    	        // REVERSE TRIM: left motor reduced slightly
		    	        uint16_t leftMotor  = LOW_SPEED;   // adjust 20–80 if needed
		    	        uint16_t rightMotor = LOW_SPEED;        // normal right motor

		    	        TIM2->CCR1 = leftMotor;   // LEFT wheel PWM
		    	        TIM2->CCR2 = rightMotor;  // RIGHT wheel PWM
		    	    }
		     }
		 }
		 else if (distanceCM > SAFE_DIST)
		 {
		     // Clear path — go straight
		     Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);

		     //uint16_t leftMotor = NORM_SPEED;
		     //uint16_t rightMotor = NORM_SPEED;   // ← adjust this value (20–80)


		     TIM2->CCR1 = NORM_SPEED;
		     TIM2->CCR2 = NORM_SPEED;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F12981;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin
                          |Front_Sensor_Pin|Left_Sensor_Pin|GPIO_PIN_8|Back_Sensor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Blue_Pin|Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_Pushbutton_Pin */
  GPIO_InitStruct.Pin = Blue_Pushbutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_Pushbutton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin
                           Front_Sensor_Pin Left_Sensor_Pin PC8 Back_Sensor_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin
                          |Front_Sensor_Pin|Left_Sensor_Pin|GPIO_PIN_8|Back_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Blue_Pin Red_Pin */
  GPIO_InitStruct.Pin = Blue_Pin|Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_Pin */
  GPIO_InitStruct.Pin = Green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Red
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  // Green
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Blue
}

void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    HAL_GPIO_WritePin(Red_Port, Red_Pin, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Green_Port, Green_Pin, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Blue_Port, Blue_Pin, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BuzzerLED(float distance_cm)
{
	 if (distance_cm <= DANGER_DIST)
		{
		  // Object very close → constant tone
		  Buzzer_Tone(2000);
		  RGB_SetColor(1,0,0);
		}
		else if ((distance_cm > DANGER_DIST) && (distance_cm <= CRICT_DIST))
		{
		  // Fast beeping
		  Buzzer_Tone(2000);
		  RGB_SetColor(1,0,0);
		  HAL_Delay(50);      // short beep
		  Buzzer_Off();
		  RGB_Off();
		  HAL_Delay(50);
		}
		else if ((distance_cm > CRICT_DIST) && (distance_cm <= SAFE_DIST))
		{
		  // Medium-speed beeping
		  Buzzer_Tone(2000);
		  RGB_SetColor(1,0,0);
		  HAL_Delay(100);
		  Buzzer_Off();
		  RGB_Off();
		  HAL_Delay(150);
		}
		else if ((distance_cm > SAFE_DIST) && (distance_cm <= 100.0f))
		{
		  // Slow beeping
		  Buzzer_Tone(2000);
		  RGB_SetColor(0,1,0);
		  HAL_Delay(150);
		  Buzzer_Off();
		  RGB_Off();
		  HAL_Delay(350);
		}
		else if (distance_cm > 100.0f)
		{
		  // Too far → silent
		  Buzzer_Off();
		  RGB_SetColor(0,0,1);
		}
}

void Trigger_Pulse(void)
{
	//------------------------Front Sensor-----------------------------//
	GPIOC -> ODR |= (1 << 6); //set PC6 high
	delayMicroseconds(10); //10 micro second pulse
	GPIOC -> ODR &= ~(1 << 6); // set PC6 low. clear pin


	//------------------------Left Sensor-----------------------------//
	GPIOC -> ODR |= (1 << 7); //set PC7 high
	delayMicroseconds(10); //10 micro second pulse
	GPIOC -> ODR &= ~(1 << 7); // set PC7 low. clear pin

	//------------------------Right Sensor-----------------------------//
	GPIOC -> ODR |= (1 << 8); //set PC8 high
	delayMicroseconds(10); //10 micro second pulse
	GPIOC -> ODR &= ~(1 << 8); // set PC8 low. clear pin


	//------------------------Rear Sensor-----------------------------//
	GPIOC -> ODR |= (1 << 9); //set PC9 high
	delayMicroseconds(10); //10 micro second pulse
	GPIOC -> ODR &= ~(1 << 9); // set PC9 low. clear pin

}

void delayMicroseconds(uint32_t microseconds)
{
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

void oled_Display(float dist_cm)
{
	   SSD1306_GotoXY(10, 10);
	   SSD1306_Puts("Distance:", &Font_11x18, 1);
	   SSD1306_GotoXY(10, 30);
	   snprintf(buf, sizeof(buf), "%.2f", dist_cm);
	   SSD1306_Puts(buf, &Font_11x18, 1);
	   SSD1306_UpdateScreen();				//update the screen to show dot
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


