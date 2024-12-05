#ifndef ULTRASONIC_FINAL_H
#define ULTRASONIC_FINAL_H
#include <avr/io.h>

typedef struct
{
    uint8_t TRIG_PIN;
    uint8_t ECHO_PIN;
    volatile float distance;
} ultrasonic_sensor;

// Global structure to manage both sensors
typedef struct
{
    ultrasonic_sensor sensor1;
    ultrasonic_sensor sensor2;
} GLOBAL_VARS;

extern GLOBAL_VARS global_state;

// calculations for degrees and distance
int16_t calculate_turn(float d1, float d2);                             // returns degrees to turn wheels
int16_t calculate_forward_distance(float d1, float d2, int16_t angle); // returns the distance forward the car needs to go
void read_distance(ultrasonic_sensor *sensor);

void TCB0_init(void);
void ultrasonic_init(void);

#endif