#include "sensors.h"
#include "main.h"

void sensors_init(){

	Sensors *sensor[NUM_SENSORS];

	for(int i = 0; i < NUM_SENSORS; i++){
		sensor[i]->last_captured = 0;
		sensor[i]->pulse_width = 0;
		sensor[i]->distance_cm = 0;
		sensor[i]->signal_polarity = 0;
	}

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
}
