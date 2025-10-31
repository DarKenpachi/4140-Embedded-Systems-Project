#include "motor_control.h"

#define CLAMP(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

void Motor_Init(){

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	Motor_Speed(0, 0);
	Motor_Direction(MOTOR_BRAKE, MOTOR_BRAKE);
}

void Motor_Speed(uint8_t speed_left, uint8_t speed_right){

	speed_left = CLAMP(speed_left, 0, 100);
	speed_right = CLAMP(speed_right, 0, 100);

	uint32_t pulse_left = (speed_left * MAX_PWM_PULSE) / 100;
	uint32_t pulse_right = (speed_right * MAX_PWM_PULSE) / 100;

	__HAL_TIM_SET_COMPARE(PWM_TIMER, LEFT_PWM_CHANNEL, pulse_left);
	__HAL_TIM_SET_COMPARE(PWM_TIMER, RIGHT_PWM_CHANNEL, pulse_right);
}
void Motor_Direction(Motor_Spin left_dir, Motor_Spin right_dir){

	switch(left_dir){

		case MOTOR_FORWARD:
			HAL_GPIO_WritePin(LEFT_IN1_PORT, LEFT_IN1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LEFT_IN2_PORT, LEFT_IN2_PIN, GPIO_PIN_RESET);
			break;

		case MOTOR_BACKWARD:
			HAL_GPIO_WritePin(LEFT_IN1_PORT, LEFT_IN1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LEFT_IN2_PORT, LEFT_IN2_PIN, GPIO_PIN_SET);
			break;

		case MOTOR_BRAKE:
			HAL_GPIO_WritePin(LEFT_IN1_PORT, LEFT_IN1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LEFT_IN2_PORT, LEFT_IN2_PIN, GPIO_PIN_SET);
			break;
		case MOTOR_COAST:
		default:
				HAL_GPIO_WritePin(LEFT_IN1_PORT, LEFT_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LEFT_IN2_PORT, LEFT_IN2_PIN, GPIO_PIN_RESET);
				break;
		}

	switch (right_dir) {

		case MOTOR_FORWARD:
			HAL_GPIO_WritePin(RIGHT_IN3_PORT, RIGHT_IN3_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RIGHT_IN4_PORT, RIGHT_IN4_PIN, GPIO_PIN_RESET);
			break;

		case MOTOR_BACKWARD:
			HAL_GPIO_WritePin(RIGHT_IN3_PORT, RIGHT_IN3_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RIGHT_IN4_PORT, RIGHT_IN4_PIN, GPIO_PIN_SET);
			break;

		case MOTOR_BRAKE:
			HAL_GPIO_WritePin(RIGHT_IN3_PORT, RIGHT_IN3_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RIGHT_IN4_PORT, RIGHT_IN4_PIN, GPIO_PIN_SET);
			break;
		case MOTOR_COAST:
		default:
			HAL_GPIO_WritePin(RIGHT_IN3_PORT, RIGHT_IN3_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RIGHT_IN4_PORT, RIGHT_IN4_PIN, GPIO_PIN_RESET);
			break;
	    }
}




