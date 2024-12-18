#ifndef ULTRASONIC_FINAL_H
#define ULTRASONIC_FINAL_H
#include <avr/io.h>

typedef struct
{
    uint8_t TRIG_PIN;
    uint8_t ECHO_PIN;
    volatile int16_t distance;
} ultrasonic_sensor;

// Global structure to manage both sensors
typedef struct
{
    ultrasonic_sensor sensor1;
    ultrasonic_sensor sensor2;
} GLOBAL_VARS;

extern GLOBAL_VARS global_state;

void read_distance(ultrasonic_sensor *sensor);

void TCB0_init(void);
void ultrasonic_init(void);

#endif