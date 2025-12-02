#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stm32l4xx_hal.h"

#define FRONT_TRIG_PORT GPIOC
#define FRONT_TRIG_PIN GPIO_PIN_6

#define LEFT_TRIG_PORT GPIOC
#define LEFT_TRIG_PIN GPIO_PIN_7

#define RIGHT_TRIG_PORT GPIOC
#define RIGHT_TRIG_PIN GPIO_PIN_8

#define BACK_TRIG_PORT GPIOC
#define BACK_TRIG_PIN GPIO_PIN_9


extern TIM_HandleTypeDef htim4;
#define NUM_SENSORS 4

typedef struct{

	volatile uint32_t last_captured;     // Last timer value captured
	volatile uint32_t pulse_width;       // Measured pulse width in microseconds
	volatile float distance_cm;       // Calculated distance
	volatile uint8_t  signal_polarity;   // 0 = waiting for rising, 1 = waiting for falling

} Sensors;

void sensors_init(Sensors*);
void sensor_handle(Sensors*, uint32_t, uint8_t);
void backTrigger();
void Trigger_Pulse();
void delayMicroseconds(uint32_t);

#endif /* INC_SENSORS_H_ */
