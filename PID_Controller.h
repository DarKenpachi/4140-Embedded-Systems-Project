#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

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
													float min_out, float max_out);

float PID_Compute(PID_Controller *pid, float current_value);

#endif /* INC_PID_CONTROLLER_H_ */
