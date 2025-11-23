/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint32_t pulse_width, pulse_width2, pulse_width3, pulse_width4;
extern volatile uint32_t signal_polarity, signal_polarity2, signal_polarity3, signal_polarity4;
extern volatile uint32_t last_captured, last_captured2, last_captured3, last_captured4;
extern volatile float distanceCM, distanceCM2, distanceCM3, distanceCM4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	uint32_t current_captured;
    uint32_t current_captured2;
    uint32_t current_captured3;
    uint32_t current_captured4;

    // --- Channel 1 ---
	if(TIM4->SR & TIM_SR_CC1IF)
	{
		current_captured = TIM4->CCR1;
		signal_polarity = 1 - signal_polarity;

		if (signal_polarity == 0) //Falling edge
		{
			//pulse_width = current_captured - last_captured
			if (current_captured >= last_captured)
				pulse_width = current_captured - last_captured;
			else
				pulse_width = (TIM4->ARR - last_captured + current_captured + 1);

			//calculate the distance immediately using 40 µs/tick
			//distanceCM = (pulse_width / 58) + 6; // assuming 1 µs tick
			distanceCM = (pulse_width * 0.0343f) / 2.0f;
		}

		last_captured = current_captured;  //store for next edge
		TIM4->SR &= ~TIM_SR_CC1IF; // clear the cc1 interrupt flag
	}

	// --- Channel 2 ---
	if(TIM4->SR & TIM_SR_CC2IF)
	{
		current_captured2 = TIM4->CCR2;
		signal_polarity2 = 1 - signal_polarity2;

		if (signal_polarity2 == 0) //Falling edge
		{
			//pulse_width = current_captured - last_captured
			if (current_captured2 >= last_captured2)
				pulse_width2 = current_captured2 - last_captured2;
			else
				pulse_width2 = (TIM4->ARR - last_captured2 + current_captured2 + 1);

			//calculate the distance immediately using 40 µs/tick
			//distanceCM = (pulse_width / 58) + 6; // assuming 1 µs tick
			distanceCM2 = (pulse_width2 * 0.0343f) / 2.0f;
		}

		last_captured2 = current_captured2;  //store for next edge
		TIM4->SR &= ~TIM_SR_CC2IF; // clear the cc1 interrupt flag
	}

	  // --- Channel 3 ---
	if(TIM4->SR & TIM_SR_CC3IF)
	{
		current_captured3 = TIM4->CCR3;
		signal_polarity3 = 1 - signal_polarity3;

		if (signal_polarity3 == 0) //Falling edge
		{
			//pulse_width = current_captured - last_captured
			if (current_captured3 >= last_captured3)
				pulse_width3 = current_captured3 - last_captured3;
			else
				pulse_width3 = (TIM4->ARR - last_captured3 + current_captured3 + 1);

			//calculate the distance immediately using 40 µs/tick
			//distanceCM = (pulse_width / 58) + 6; // assuming 1 µs tick
			distanceCM3 = (pulse_width3 * 0.0343f) / 2.0f;
		}

		last_captured3 = current_captured3;  //store for next edge
		TIM4->SR &= ~TIM_SR_CC3IF; // clear the cc1 interrupt flag
	}

	  // --- Channel 4 ---
	if(TIM4->SR & TIM_SR_CC4IF)
	{
		current_captured4 = TIM4->CCR4;
		signal_polarity4 = 1 - signal_polarity4;

		if (signal_polarity4 == 0) //Falling edge
		{
			//pulse_width = current_captured - last_captured
			if (current_captured4 >= last_captured4)
				pulse_width4 = current_captured4 - last_captured4;
			else
				pulse_width4 = (TIM4->ARR - last_captured4 + current_captured4 + 1);

			//calculate the distance immediately using 40 µs/tick
			//distanceCM = (pulse_width / 58) + 6; // assuming 1 µs tick
			distanceCM4 = (pulse_width4 * 0.0343f) / 2.0f;
		}

		last_captured4 = current_captured4;  //store for next edge
		TIM4->SR &= ~TIM_SR_CC4IF; // clear the cc1 interrupt flag
	}

    // --- Overflow ---
    if (TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;
    }
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
