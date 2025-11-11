#include "PID_Controller.h"

#define CLAMP(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint,
													int8_t min_out, int8_t max_out){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->setpoint = setpoint;
	pid->prev_error = 0.0f;
	pid->integral = 0.0f;
	pid->min_output = min_out;
	pid->max_output = max_out;
}

int8_t PID_Compute(PID_Controller *pid, float current_value, float delta){

	float error = pid->setpoint - current_value;

	float p = pid->Kp * error;

	pid->integral += error * delta;
	pid->integral = CLAMP(pid->integral, pid->min_output / pid->Ki, (pid->max_output / pid->Ki));
	float i = pid->Ki * pid->integral;

	float derivative = (error - pid->prev_error) / delta;
	float d = pid->Kd * derivative;

	float output = p + i + d;

	output = CLAMP(output, pid->min_output, pid->max_output);

	pid->prev_error = error;

	return output;
}

