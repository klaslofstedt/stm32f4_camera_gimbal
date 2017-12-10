#ifndef JOYSTICK_H
#define JOYSTICK_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"


typedef struct{
    uint8_t channel;
    uint16_t raw_value;
    uint16_t deadband;
    uint16_t middle;
    uint16_t raw_min;
    uint16_t raw_max;
    float threshold;
    float step_size;
    float setpoint;
    float min;
    float max;
    float value;
} joystick_data_t;

void joystick_init(void);
uint16_t joystick_read_raw(uint8_t channel);
float joystick_read(joystick_data_t *joystick);

#endif