#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "move.h"
#include "ultrasonic_final.h"

#define MULTIPLIER 100 //to help integer calculation
#define CYCLE 10 //2x10ms

#define MAX_SPEED (20 * MULTIPLIER) // cm/sec
#define MIN_SPEED (-20 * MULTIPLIER)
#define DIR_CHG_SPEED (MAX_SPEED / 45)
#define MAX_ANGLE 45
#define MIN_ANGLE -45

#define PULSES_PER_CM 0.43 * MULTIPLIER // 20.5cm / 48 pulse = 0.43 cm/pulse

#define TIMER_PERIOD (33333) // Generating interupt every 10ms

volatile uint16_t left_encoder_count = 0;
volatile uint16_t right_encoder_count = 0;

volatile int16_t sensor_total_angle = 0;
volatile int16_t sensor_speed = 0;
volatile uint8_t sensor_readings_ready = 0;

volatile uint8_t counter_10ms = 0;

volatile int32_t current_speed = 0;
volatile int32_t current_angle = 0;

volatile float sensor_dist1 = 0;
volatile float sensor_dist2 = 0;
    
void TCA0_init(void)
{
    TCA0.SINGLE.PER = TIMER_PERIOD;           // 10ms
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;   // Overflow interrupt
                          TCA_SINGLE_CMP0_bm; // Compare 0 interrupt PWMA
                          TCA_SINGLE_CMP1_bm; // Compare 1 interrupt PWMB

    // Enable CMP0 and CMP1 for output and set single-slope PWM mode
    TCA0.SINGLE.CTRLB = 
            TCA_SINGLE_CMP0EN_bm | 
            TCA_SINGLE_CMP1EN_bm | 
            TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
            // TCA_SINGLE_WGMODE_NORMAL_gc;
    
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}


void encoder_init(void)
{
    PORTD.DIRCLR = PIN5_bm | PIN7_bm;    // PD5 (Encoder A), PD7 (Encoder B) as input
    PORTD.PIN5CTRL |= PORT_ISC_RISING_gc;
    PORTD.PIN7CTRL |= PORT_ISC_RISING_gc;
}

void motor_init(void)
{
    // Getting the PWM outputs on port C
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;

    // no idea which one is left or right
    PORTC.DIR |= PIN1_bm; // PWMA -> M2
    PORTC.DIR |= PIN0_bm; // PWMB -> M1
    PORTD.DIR |= PIN1_bm; // AIN2+BIN2
    PORTD.DIR |= PIN6_bm; // AIN1+BIN1

    TCA0.SINGLE.CMP0BUF = 0;
    TCA0.SINGLE.CMP1BUF = 0;
}


// ENCODER INTERRUPTS
// ISR for Encoder A (PD5) and Encoder B (PD7) ??? right or left
ISR(PORTD_PORT_vect)
{
    if(PORTD.INTFLAGS & PIN5_bm) { // assuming this is left
        PORTD.INTFLAGS = PIN5_bm;
        left_encoder_count++;
    }
    if(PORTD.INTFLAGS & PIN7_bm) { // assuming this is right
        PORTD.INTFLAGS = PIN7_bm;
        right_encoder_count++;
    }
}

// TIMER INTERRUPTS
ISR(TCA0_OVF_vect) // every 10ms
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    counter_10ms++;

    if (counter_10ms == CYCLE) {
        set_motor_speed(sensor_speed, sensor_total_angle);
        left_encoder_count = 0;
        right_encoder_count = 0;
        counter_10ms = 0;
        sensor_readings_ready = 1;
    }
}

// every 100ms
// speed : mid point speed of the car (between two tires)
// left_speed = speed + n*angle : if angle is + : turn right
// right_speed = speed - n*angle
// n=0.05 * 45 degree = 2.25 max speed increase/decrease for a tire
void set_motor_speed(int16_t target_speed, int16_t target_angle)
{
    // Bound checks
    if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
    if (target_speed < MIN_SPEED) target_speed = MIN_SPEED;
    if (target_angle > MAX_ANGLE) target_angle = MAX_ANGLE;
    if (target_angle < MIN_ANGLE) target_angle = MIN_ANGLE;

    int8_t dir = 1;
    if (target_speed<0) {
        dir = -1;
        target_speed = -target_speed;
        target_angle = -target_angle;
    }

    // pwm values using the speed information

    // encoders feedback
    uint16_t actual_speed = (left_encoder_count+right_encoder_count) / 2 * MULTIPLIER * (100/CYCLE) / PULSES_PER_CM;  // speed in cm/sec
    
    if (current_speed < target_speed) current_speed += 2 * MULTIPLIER;
    if (current_speed > target_speed) current_speed -= 2 * MULTIPLIER;
    
    // more bound checks
    if (current_speed > MAX_SPEED) current_speed = MAX_SPEED;
    if (current_speed < MIN_SPEED) current_speed = MIN_SPEED;
    
    if (current_angle < target_angle) current_angle += 2;
    if (current_angle > target_angle) current_angle -= 2;
    
    // more bound checks
    if (current_angle > MAX_ANGLE) current_angle = MAX_ANGLE;
    if (current_angle < MAX_ANGLE) current_angle = MAX_ANGLE;

    //--------------------------------------------------------------
    //Use divider 2 to slow down car
    int32_t pwm0 = (int32_t)(current_speed + DIR_CHG_SPEED * target_angle) * TIMER_PERIOD / MAX_SPEED / 2;
    int32_t pwm1 = (int32_t)(current_speed - DIR_CHG_SPEED * target_angle) * TIMER_PERIOD / MAX_SPEED / 2;
 
    //Logaritmic scale : 0 -> 25%
    if (pwm0 > 0) pwm0 = pwm0 + (TIMER_PERIOD - pwm0) / 3;
    if (pwm1 > 0) pwm1 = pwm1 + (TIMER_PERIOD - pwm1) / 3;
    
    if (pwm0 > TIMER_PERIOD) pwm0 = TIMER_PERIOD;
    if (pwm0 < 0) pwm0 = 0;
    if (pwm1 > TIMER_PERIOD) pwm1 = TIMER_PERIOD;
    if (pwm1 < 0) pwm1 = 0;
    
    TCA0.SINGLE.CMP0BUF = pwm0;
    TCA0.SINGLE.CMP1BUF = pwm1;
    // Set direction for Motor A-B
    if (dir == 1) {
        //forward
        PORTD.OUTSET = PIN1_bm; // AIN1/BIN1 high for forward
        PORTD.OUTCLR = PIN6_bm; // AIN2/BIN2 low for forward
    } else {
        PORTD.OUTSET = PIN6_bm; // AIN2/BIN2 high for backward
        PORTD.OUTCLR = PIN1_bm; // AIN1/BIN1 low for backward
    }
}

int main(void)
{
    TCA0_init();
    encoder_init();
    motor_init();
    ultrasonic_init(); // Initialize GPIO for ultrasonic sensor
    TCB0_init();

    sei(); // Enable global interrupts
    
    while (1)
    {
        if (sensor_readings_ready) {
            // Measure distances for both sensors
            read_distance(&global_state.sensor1);
            read_distance(&global_state.sensor2);

            sensor_dist1 = global_state.sensor1.distance;
            if (sensor_dist1 == -1) continue;
            
            sensor_dist2 = global_state.sensor2.distance;
            if (sensor_dist2 == -1) continue;
            
//            sensor_total_angle = 0;
//            sensor_speed = 5 * MULTIPLIER;
            sensor_total_angle = calculate_turn(sensor_dist1, sensor_dist2);
            sensor_total_angle = 0;
            sensor_speed = calculate_forward_distance(sensor_dist1, sensor_dist2, sensor_total_angle) * MULTIPLIER;
            sensor_readings_ready = 0;
        }
    }
}