#include "sensors.h"
#define CLAMP(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

void sensors_init(Sensors *sensor){

	for(int i = 0; i < NUM_SENSORS; i++){
		sensor[i].last_captured = 0;
		sensor[i].pulse_width = 0;
		sensor[i].distance_cm = 0;
		sensor[i].signal_polarity = 0;
	}

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
}
void sensor_handle(Sensors *sensor, uint32_t ccr_val, uint8_t i){

    if (sensor[i].signal_polarity == 0) // RISING EDGE
    {
        sensor[i].last_captured = ccr_val;
        sensor[i].signal_polarity = 1;
    }
    else // FALLING EDGE
    {
        sensor[i].pulse_width = ccr_val - sensor[i].last_captured;

        if (ccr_val < sensor[i].last_captured) {

            sensor[i].pulse_width = (65535 - sensor[i].last_captured) + ccr_val;
        }

        uint32_t clamped_pulse = CLAMP(sensor[i].pulse_width, 2 * 58.0f, 400 * 58.0f);
        sensor[i].distance_cm = (float)clamped_pulse / 58.0f;
        //sensor[i].distance_cm = (sensor[i].pulse_width / 58.0f);
        //sensor[i].distance_cm = CLAMP(sensor[i].distance_cm, 2.0f, 400.0f);

        sensor[i].signal_polarity = 0;
    }
}

void Trigger_Pulse(void){

	int i;
	for(i = 0; i < NUM_SENSORS; i++){

		if(i == 0){

			HAL_GPIO_WritePin(FRONT_TRIG_PORT, FRONT_TRIG_PIN, GPIO_PIN_SET);
			delayMicroseconds(10); // 10us pulse
			HAL_GPIO_WritePin(FRONT_TRIG_PORT, FRONT_TRIG_PIN, GPIO_PIN_RESET);
		}
		else if(i == 1){

			HAL_GPIO_WritePin(LEFT_TRIG_PORT, LEFT_TRIG_PIN, GPIO_PIN_SET);
			delayMicroseconds(10); // 10us pulse
			HAL_GPIO_WritePin(LEFT_TRIG_PORT, LEFT_TRIG_PIN, GPIO_PIN_RESET);
		}
		else if(i == 2){

			HAL_GPIO_WritePin(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, GPIO_PIN_SET);
			delayMicroseconds(10); // 10us pulse
			HAL_GPIO_WritePin(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, GPIO_PIN_RESET);
		}
		else if(i == 3){

			HAL_GPIO_WritePin(BACK_TRIG_PORT, BACK_TRIG_PIN, GPIO_PIN_SET);
			delayMicroseconds(10);
			HAL_GPIO_WritePin(BACK_TRIG_PORT, BACK_TRIG_PIN, GPIO_PIN_RESET);
		}
		HAL_Delay(20);
	}
}

void delayMicroseconds(uint32_t microseconds){

	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < microseconds)
	{
		// Busy-wait
	}

}
