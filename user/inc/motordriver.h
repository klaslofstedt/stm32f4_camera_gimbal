#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <stdbool.h>

typedef struct{
    bool dir;
    uint32_t step_size;
    uint32_t cycle_size;
    int32_t step;
    uint16_t reminder;
    float power; // 0-1
}driver_data_t;

void motordriver_init(void);
void motordriver_set2(uint32_t i, int32_t m_delay, float damper);
void motordriver_set1(uint32_t i, int32_t m_delay, float damper);



#endif