#ifndef MOVE_H
#define MOVE_H
#include <avr/io.h>

// sensors should be changing these periodically
extern volatile int16_t sensor_total_angle;
extern volatile int16_t sensor_speed; // cm/sec

void motor_init(void);
void encoder_init(void);
void TCA0_init(void);

#endif
