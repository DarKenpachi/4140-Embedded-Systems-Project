#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32l4xx_hal.h"

#define LEFT_IN1_PORT 	GPIOC
#define LEFT_IN1_PIN 	GPIO_PIN_0
#define LEFT_IN2_PORT 	GPIOC
#define LEFT_IN2_PIN 	GPIO_PIN_1

#define RIGHT_IN3_PORT 	GPIOC
#define RIGHT_IN3_PIN 	GPIO_PIN_2
#define RIGHT_IN4_PORT 	GPIOC
#define RIGHT_IN4_PIN  	GPIO_PIN_3

extern TIM_HandleTypeDef htim2;
#define PWM_TIMER  &htim2
#define LEFT_PWM_CHANNEL	TIM_CHANNEL_1
#define RIGHT_PWM_CHANNEL	TIM_CHANNEL_2

#define MAX_PWM __HAL_TIM_GET_AUTORELOAD(PWM_TIMER)

typedef enum{
	MOTOR_COAST,
	MOTOR_BRAKE,
	MOTOR_FORWARD,
	MOTOR_BACKWARD
} Motor_Spin;

void Motor_Init(void);
void Motor_Speed(int16_t speed_left, int16_t speed_right);
void Motor_Direction(Motor_Spin left_dir, Motor_Spin right_dir);

#endif /* INC_MOTOR_CONTROL_H_ */
