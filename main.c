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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Clamps values if out of bounds
#define CLAMP(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

//Rover parameters
#define MAX_SPEED 95
#define MIN_SPEED 45
#define SAFE_DIST 35.0f
#define CRITICAL_DIST 12.5f
#define SETPOINT 0.0f

//Steering PID gains
#define KP_Steer 1.5f
#define KI_Steer 0.7f
#define KD_Steer 0.2f

//Speed PID gains
#define KP_Speed 4.9f
#define KI_Speed 1.5f
#define KD_Speed 0.3f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;
DMA_HandleTypeDef hdma_tim4_ch3;

/* USER CODE BEGIN PV */
volatile float front_cm;
volatile float left_cm;
volatile float right_cm;
volatile float back_cm;
volatile float back_dist_traveled = 0.0f;
volatile float current_back_dist = 0.0f;
volatile float prev_back_dist = 0.0f;
volatile float centering_err = 0.0f;
volatile float steering_adj  = 0.0f;
volatile int8_t base_speed = 0;
volatile int8_t speed_adj = 0;
volatile int8_t speed_err = 0;
volatile int8_t left_speed  = 0;
volatile int8_t right_speed = 0;
volatile int8_t current_right_speed = 0;
volatile int8_t current_left_speed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
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
  /* USER CODE BEGIN 2 */
  PID_Controller pid_steer;
  PID_Controller pid_speed;

  Motor_Init();
  sensors_init(&sensor);
  PID_Init(&pid_steer, KP_Steer, KI_Steer, KD_Steer, SETPOINT, -MIN_SPEED, MAX_SPEED);
  PID_Init(&pid_speed, KP_Speed, KI_Speed, KD_Speed, SAFE_DIST, -MIN_SPEED, MAX_SPEED);
  uint32_t last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Trigger_Pulse();

	  front_cm = sensor[0].distance_cm;
	  left_cm  = sensor[1].distance_cm;
	  right_cm = sensor[2].distance_cm;
	  back_cm  = sensor[3].distance_cm;

	  HAL_Delay(10);

	  uint32_t current_tick = HAL_GetTick();
	  float delta = (current_tick - last_tick) / 1000.0f;
	  last_tick = current_tick;

	  centering_err = left_cm - right_cm;
	  speed_err = SAFE_DIST - front_cm;

	  base_speed = PID_Compute(&pid_speed, speed_err, delta);
	  steering_adj = PID_Compute(&pid_steer, centering_err, delta);

	  left_speed = base_speed + steering_adj;
	  right_speed = base_speed - steering_adj;

	  if(left_speed < -MAX_SPEED){
		  left_speed = -MAX_SPEED;
	  }
	  else if(left_speed > MAX_SPEED){
		  left_speed = MAX_SPEED;
	  }

	  if(right_speed < -MAX_SPEED){
		  right_speed = -MAX_SPEED;
	  }
	  else if(right_speed > MAX_SPEED){
		  right_speed = MAX_SPEED;
	  }

	  if((left_speed < 0) && (right_speed > 0)){

		  Motor_Direction(MOTOR_BACKWARD, MOTOR_FORWARD);
		  HAL_Delay(10);
	  }
	  else if((left_speed > 0) && (right_speed < 0)){

		  Motor_Direction(MOTOR_FORWARD, MOTOR_BACKWARD);
		  HAL_Delay(25);
	  }
	  else if((left_speed < 0) && (right_speed < 0)){

		  Motor_Direction(MOTOR_BACKWARD, MOTOR_BACKWARD);
		  HAL_Delay(10);
	  }
	  else {
		  Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);
		  HAL_Delay(10);
	  }
	  TIM2->CCR1 = abs(left_speed);
	  TIM2->CCR2 = abs(right_speed);
	  //Motor_Speed(left_speed, right_speed);
	  HAL_Delay(50);
	  /*
	  if(front_cm < CRITICAL_DIST){

		  Motor_Direction(MOTOR_BRAKE, MOTOR_BRAKE);
		  HAL_Delay(200);

		  if((back_cm > 5.0f) || (back_dist_traveled < 25.0f)){

			  HAL_GPIO_WritePin(BACK_TRIG_PORT, BACK_TRIG_PIN, GPIO_PIN_SET);
			  delayMicroseconds(10);
			  HAL_GPIO_WritePin(BACK_TRIG_PORT, BACK_TRIG_PIN, GPIO_PIN_RESET);

			  HAL_Delay(20);

			  back_cm = sensor[3].distance_cm;

			  Motor_Direction(MOTOR_BACKWARD, MOTOR_BACKWARD);
			  Motor_Speed(BASE_SPEED, BASE_SPEED);

			  current_back_dist = back_cm;
			  back_dist_traveled += prev_back_dist - current_back_dist;
			  prev_back_dist = current_back_dist;
		  }

		  Motor_Direction(MOTOR_BRAKE, MOTOR_BRAKE);
		  Motor_Speed(0,0);
		  HAL_Delay(200);
	  }
	  else if(front_cm < SAFE_DIST){

		  if(left_cm > right_cm){

			  Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);
			  Motor_Speed(MIN_SPEED, BASE_SPEED);
			  HAL_Delay(600);
		  }

		  else if(right_cm > left_cm){

			  Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);
			  Motor_Speed(BASE_SPEED, MIN_SPEED);
			  HAL_Delay(600);
		  }

		  Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);
		  steering_adj = PID_Compute(&pid_steer, centering_err, delta);
		  speed_adj = PID_Compute(&pid_speed, speed_err, delta);

		  left_speed = BASE_SPEED + steering_adj;
		  right_speed = BASE_SPEED - steering_adj;

		  Motor_Speed(left_speed, right_speed);
	  }
	  else{

		  Motor_Direction(MOTOR_FORWARD, MOTOR_FORWARD);

		  speed_adj = PID_Compute(&pid_speed, speed_err, delta);

		  left_speed = current_left_speed + speed_adj;
		  right_speed = current_right_speed - speed_adj;

		  if(left_speed < 0){

			  Motor_Direction(MOTOR_BACKWARD, MOTOR_FORWARD);
			  HAL_Delay(50);
		  }
		  else if(right_speed < 0){

			  Motor_Direction(MOTOR_FORWARD, MOTOR_BACKWARD);
			  HAL_Delay(50);
		  }

		  Motor_Speed(left_speed, right_speed);

		  steering_adj = PID_Compute(&pid_steer, centering_err, delta);

		  left_speed = current_left_speed + steering_adj;
		  right_speed = current_left_speed - steering_adj;
		  Motor_Speed(left_speed, right_speed);

		  current_left_speed = left_speed;
		  current_right_speed = right_speed;

		  HAL_Delay(10);
	  }*/
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin
                          |Front_Sensor_Pin|Left_Sensor_Pin|GPIO_PIN_8|Back_Sensor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin
                           Front_Sensor_Pin Left_Sensor_Pin PC8 Back_Sensor_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin
                          |Front_Sensor_Pin|Left_Sensor_Pin|GPIO_PIN_8|Back_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
