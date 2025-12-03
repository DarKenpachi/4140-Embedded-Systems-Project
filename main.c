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
#include "PID_Controller.h"
#include "motor_control.h"
#include "sensors.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Clamps values if out of bounds
#define CLAMP(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

//Rover parameters
#define MAX_SPEED 80
#define SAFE_DIST 65.0f
#define SETPOINT 0.0f

//Steering PID gains
#define KP_Steer 0.62f
#define KI_Steer 0.0f
#define KD_Steer 0.0375f

//Speed PID gains
#define KP_Speed 0.92f
#define KI_Speed 0.12f
#define KD_Speed 0.0f

#define ALPHA 0.9f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;
DMA_HandleTypeDef hdma_tim4_ch3;

/* USER CODE BEGIN PV */
volatile float front_cm;
volatile float left_cm;
volatile float right_cm;
volatile float back_cm;

volatile float centering_err = 0.0f;
volatile float steering_adj  = 0.0f;

volatile float front_filtered = 0.0f;
volatile float right_filtered = 0.0f;
volatile float left_filtered = 0.0f;

volatile float prev_front_filtered = 0.0f;
volatile float prev_right_filtered = 0.0f;
volatile float prev_left_filtered = 0.0f;

volatile int16_t base_speed = 0;
volatile int16_t speed_err = 0;
volatile int16_t left_speed  = 0;
volatile int16_t right_speed = 0;
volatile int8_t flag = 0;

static uint32_t counter = 0;

extern Sensors sensor[NUM_SENSORS];

GPIO_PinState button_state;
volatile GPIO_PinState Red;
volatile GPIO_PinState Green;
volatile GPIO_PinState Blue;

//------------------ Daniel's change. Add a state variable for the LED/Buzzer toggle
volatile int8_t led_buzzer_state = 0; // 0 for OFF, 1 for ON

uint16_t blink_interval_ticks = 0;
int8_t buzzer_enabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void RGB_SetColor( uint8_t, uint8_t, uint8_t);
void RGB_Off();
void buzzer_RGB(float);
void oled_Display(float);

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
  /* USER CODE BEGIN 2 */
  PID_Controller pid_steer;
  PID_Controller pid_speed;

  Motor_Init();
  sensors_init(sensor);
  PID_Init(&pid_steer, KP_Steer, KI_Steer, KD_Steer, SETPOINT, -MAX_SPEED, MAX_SPEED);
  PID_Init(&pid_speed, KP_Speed, KI_Speed, KD_Speed, SAFE_DIST, -MAX_SPEED, MAX_SPEED);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  uint32_t last_tick = HAL_GetTick();
  uint8_t button_flag = 0;
  uint8_t first_run = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	  if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) && (button_state == GPIO_PIN_RESET)){
		  button_flag = 1;
	  }
	  // starts on button press
	  if(button_flag){

		  Trigger_Pulse();

		  front_cm = sensor[0].distance_cm;
		  left_cm  = sensor[1].distance_cm;
		  right_cm = sensor[2].distance_cm;

		  if(first_run) {
			  front_filtered = front_cm;
			  left_filtered = left_cm;
			  right_filtered = right_cm;

			  first_run = 0;
		  }
		  else {
			  //filters the signal noise to get accurate readings
			  front_filtered = (ALPHA * front_cm) + ((1 - ALPHA) * prev_front_filtered);
			  left_filtered  = (ALPHA * left_cm) + ((1 - ALPHA) * prev_left_filtered);
			  right_filtered = (ALPHA * right_cm) + ((1 - ALPHA) * prev_right_filtered);
		  }
		  buzzer_RGB(front_filtered);

		  uint32_t current_tick = HAL_GetTick();
		  float delta = (float)(current_tick - last_tick) / 1000.0f;
		  last_tick = current_tick;

		  //error to feed PID to calculate steering and speed output
		  centering_err = left_filtered - right_filtered;
		  speed_err =  front_filtered - SAFE_DIST;

		  base_speed = PID_Compute(&pid_speed, speed_err, delta, 1);
		  steering_adj = PID_Compute(&pid_steer, centering_err, delta, 0);

		  base_speed = CLAMP(base_speed, 0, MAX_SPEED);

		  left_speed = CLAMP((base_speed + steering_adj), -MAX_SPEED, MAX_SPEED);
		  right_speed = CLAMP((base_speed - steering_adj), -MAX_SPEED, MAX_SPEED);

		  //if statements to determine direction from PID output
		  if(left_speed < 0 && right_speed > 0){
			  Motor_Direction(MOTOR_BACKWARD, MOTOR_FORWARD);
		  }
		  else if(left_speed > 0 && right_speed < 0){
			  Motor_Direction(MOTOR_FORWARD, MOTOR_BACKWARD);
		  }
		  else if((left_speed < 0 && right_speed < 0)){
			  Motor_Direction(MOTOR_BACKWARD, MOTOR_BACKWARD);
		  }
		  else {
		  	  Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);
		  }
		 Motor_Speed(left_speed, right_speed);

		 prev_front_filtered = front_filtered;
		 prev_right_filtered = right_filtered;
		 prev_left_filtered = left_filtered;
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
  htim2.Init.Prescaler = 39;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
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
  htim3.Init.Prescaler = 39;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
                          |Front_Sensor_Pin|Left_Sensor_Pin|Right_Sensor_Pin|Back_Sensor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Blue_Pin|Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PushButton_Pin */
  GPIO_InitStruct.Pin = PushButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PushButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin
                           Front_Sensor_Pin Left_Sensor_Pin Right_Sensor_Pin Back_Sensor_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin
                          |Front_Sensor_Pin|Left_Sensor_Pin|Right_Sensor_Pin|Back_Sensor_Pin;
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

void RGB_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Red
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  // Green
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Blue
}

void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void buzzer_RGB(float distance_cm)
{
	if (distance_cm < 25.0f) {
		blink_interval_ticks = 100;

		Red = GPIO_PIN_SET;
		Green = GPIO_PIN_RESET;
		Blue = GPIO_PIN_RESET;

		buzzer_enabled = 1;
	}
	else if (distance_cm >= 25.0f && distance_cm < 45.0f) {
		blink_interval_ticks = 300;

		Red = GPIO_PIN_SET;
		Green = GPIO_PIN_RESET;
		Blue = GPIO_PIN_RESET;

		buzzer_enabled = 1;
	}
	else if (distance_cm > 45.0f && distance_cm < SAFE_DIST) {

		blink_interval_ticks = 500;

		Red = GPIO_PIN_SET;
		Green = GPIO_PIN_SET;
		Blue = GPIO_PIN_RESET;

		buzzer_enabled = 1;
	}
	else {

		blink_interval_ticks = 0;
		buzzer_enabled = 0;

		Red = GPIO_PIN_RESET;
		Green = GPIO_PIN_SET;
		Blue = GPIO_PIN_RESET;
	}
}
//interrupt driven timers to help minimize CPU latency
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	/*****Sharif original code****************
  if (htim->Instance == TIM3){
	  if(buzzer_enabled == 1 && counter < blink_interval_ticks){
		counter++;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    	RGB_SetColor(Red, Green, Blue);
    	TIM3->CCR1 = blink_interval_ticks;

    	if(counter >= blink_interval_ticks){
    		buzzer_enabled = 0;
    		counter = 0;
    		blink_interval_ticks = 0;
    		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    		RGB_Off();
    	}
	 }
	}*/

	/*********Daniel's code *************/
	if(htim -> Instance == TIM3){
		//check if the warning system is enabled
		if(buzzer_enabled == 1 && blink_interval_tick > 0){
			counter++;

			if(counter >= blink_interval_ticks){
				//toggle the state
				if(led-buzzer_state == 0){
					//turn ON (set color and start pwm0
					RGC_setColor(Red, Green< Blue);
					//start buzzer pwm
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
					// Set PWM duty cycle (CCR1) to a non-zero value for sound
					// Assuming 50% duty cycle
					TIM3 -> CCR1 = htim3.Instance -> ARR / 2;
					led_buzzer_state = 1;
				} else {
					/// turn off
					RGB_Off();
					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); //stop buzzer pwm
					led_buzzer_state = 0;
				}
				// reset counter for the next toggle period
				counter = 0;
			}
		} else {
			//If buzzer is not enabled (distance is safe), ensure everything is off
            RGB_Off();
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            led_buzzer_state = 0; // Ensure state is tracked as off
            counter = 0; // Reset counter for when it's re-enabled
		}
	}
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

