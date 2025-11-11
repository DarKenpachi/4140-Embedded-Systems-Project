#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_
#include <stdint.h>

typedef struct{

	//Controller gains
	float Kp;
	float Ki;
	float Kd;

	//target setpoint
	float setpoint;

	//error tracking
	float prev_error;
	float integral;

	//output limits
	float min_output;
	float max_output;

} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint,
													int8_t min_out, int8_t max_out);

int8_t PID_Compute(PID_Controller *pid, float current_value, float delta);

#endif /* INC_PID_CONTROLLER_H_ */
