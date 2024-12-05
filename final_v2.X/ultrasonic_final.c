#define F_CPU 3333333

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "ultrasonic_final.h"

GLOBAL_VARS global_state = {
    .sensor1 = {.TRIG_PIN = PIN4_bm, .ECHO_PIN = PIN5_bm, .distance = 0},
    .sensor2 = {.TRIG_PIN = PIN6_bm, .ECHO_PIN = PIN7_bm, .distance = 0}};

// needed for time to calculate how long echo was high
volatile uint16_t pulse_width = 0;


void TCB0_init(void)
{
    TCB0.CCMP =  65535; //0.3us * 64k = 19.6ms max count
    TCB0.CTRLB = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
}

// Initialize GPIO for ultrasonic sensors
void ultrasonic_init(void)
{
    // Configure pins for Sensor 1
    PORTA.DIRSET = global_state.sensor1.TRIG_PIN; // Set Trig as output
    PORTA.DIRCLR = global_state.sensor1.ECHO_PIN; // Set Echo as input

    // Configure pins for Sensor 2
    PORTA.DIRSET = global_state.sensor2.TRIG_PIN; // Set Trig as output
    PORTA.DIRCLR = global_state.sensor2.ECHO_PIN; // Set Echo as input
}

// utils 
float clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    return value;
}

uint16_t dur = 0;
// delay in microseconds using the existing timer
void my_delay_us(uint16_t delay_us) {
    uint16_t delay_pulse = delay_us * 3;
    TCB0.CNT = 0;
    dur = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
    // Wait until the specified delay in microseconds is reached
    while (TCB0.CNT < delay_pulse);
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
}

// Read distance from the ultrasonic sensor
void read_distance(ultrasonic_sensor *sensor)
{   
    const uint16_t timeout_us = 4096 * 3; 
    float distance = 0;
    //vTaskSuspendAll(); //-------------------------
    
    // Send a 10us pulse to the Trig pin
    PORTA.OUTSET = sensor->TRIG_PIN;
    my_delay_us(10);
    PORTA.OUTCLR = sensor->TRIG_PIN;

    // Timeout if echo never goes high
    TCB0.CNT = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Start timer
    while (!(PORTA.IN & sensor->ECHO_PIN)) {
        if (TCB0.CNT >= timeout_us) {
            TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // Stop timer
            // Error in sensing 
            goto exit;
        }
    }
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // Stop timer
    TCB0.CNT = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Start timer
    while (PORTA.IN & sensor->ECHO_PIN) { // Wait for Echo pin to go low
        if (TCB0.CNT >= timeout_us) {
            TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // Stop timer
            // Error in sensing 
            goto exit;
        }
    }
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // Stop timer
    pulse_width = TCB0.CNT / 3; //usec
    
    //xTaskResumeAll(); //-------------------------

    // Calculate distance in cm
    // 10 = x * 0.34 / 2 then x = 20/0.34
    // 10cm = 59us = 177 pulse
    distance = ((float)pulse_width * 0.34) / 2.0;
    distance = clamp(distance, 0.0f, 20.0f) ; 
    if (distance > 15) distance = 0;
    if (distance > 1 && distance < 5) distance = 5 - distance;
  
 exit:
    // Update sensor with distance reading
    sensor->distance = distance;
    my_delay_us(100);
}


int16_t calculate_turn(float d1, float d2)
{
    // takes in 2 distances from sensors and computes degrees to turn
    float L = 3.0f; // l is distance between sensors
    float radians = atanf((d1 - d2) / L);
    float degrees = radians * (180.0f / M_PI);
    return (int16_t) clamp(degrees, -45.0, 45.0);
}

int16_t calculate_forward_distance(float d1, float d2, int16_t angle)
{
    float angle_radians = angle * (M_PI / 180.0f); // Convert to radians
    // scale via cos
    float distance = ((d1 + d2) / 2.0f) * cosf(angle_radians);
    // if the angle to turn is close to 0 we shouldn't move too much
    distance -= 5.0f;
    return (int16_t) clamp(distance, 0.0f, 10.0f); // for now robot shouldnt need to travel more than 10 cm/sec
}