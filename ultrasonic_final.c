#define F_CPU 3333333

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <string.h>
#include <math.h>


#define SAMPLES_PER_BIT 16
#define USART2_BAUD_VALUE(BAUD_RATE) (uint16_t)((F_CPU << 6) / (((float)SAMPLES_PER_BIT) * (BAUD_RATE)) + 0.5)

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

GLOBAL_VARS global_state = {
    .sensor1 = {.TRIG_PIN = PIN6_bm, .ECHO_PIN = PIN7_bm, .distance = 0},
    .sensor2 = {.TRIG_PIN = PIN4_bm, .ECHO_PIN = PIN5_bm, .distance = 0}};

// USART functions
void USART2_INIT(void);
void USART2_PRINTF(char *str);

// Ultrasonic functions
void ultrasonic_init(void);
void read_distance(ultrasonic_sensor *sensor);
void sensing_task(void); // reads data from both ultrasonic sensors and updates the global state with sensor readings

// needed for time to calculate how long echo was high
volatile uint16_t pulse_width = 0;
volatile uint16_t timer_count = 0;
volatile uint8_t measure_active = 0;

// Timer functions
void TCA0_init(void);
void start_timer(void);
void stop_timer(void);

// calculations for degrees and distance
float calculate_turn(float d1, float d2);                          // returns degrees to turn wheels
float calculate_forward_distance(float d1, float d2, float angle); // returns the distance forward the car needs to go

// utils 
float clamp(float value, float min, float max);

int main(void)
{
    USART2_INIT();     // Initialize USART2
    ultrasonic_init(); // Initialize GPIO for ultrasonic sensor
    TCA0_init();       // Initialize TimerB for pulse width measurement

    char buffer[100]; // Buffer for storing distance output

    sei();

    while (1)
    {
        // Measure and print distances for both sensors
        read_distance(&global_state.sensor1);
        read_distance(&global_state.sensor2);

        float sensor_dist1 = global_state.sensor1.distance;
        float sensor_dist2 = global_state.sensor2.distance;
        float turn_angle = calculate_turn(sensor_dist1, sensor_dist2);
        float dist_forward = calculate_forward_distance(sensor_dist1, sensor_dist2, turn_angle);

        // Print sensor distances, turn angle, and forward distance
        snprintf(buffer, sizeof(buffer),
                 "S1: %.2f cm | S2: %.2f cm | Turn: %.2f deg | Forward: %.2f cm\r\n",
                 sensor_dist1, sensor_dist2, turn_angle, dist_forward);
        USART2_PRINTF(buffer);

        // don't really need delay there's enough from the pin if there's no sound back echo timesout after 36ms
    }
}

// Initialize USART2
void USART2_INIT(void)
{
    PORTF.DIRSET = PIN0_bm;                // Set TX (PF0) as output
    PORTF.DIRCLR = PIN1_bm;                // Set RX (PF1) as input
    USART2.BAUD = USART2_BAUD_VALUE(9600); // Set baud rate to 9600
    USART2.CTRLB |= USART_TXEN_bm;         // Enable TX
}

// Send a string over USART2
void USART2_PRINTF(char *str)
{
    for (size_t i = 0; i < strlen(str); i++)
    {
        while (!(USART2.STATUS & USART_DREIF_bm))
            ; // Wait for the buffer to be ready
        USART2.TXDATAL = str[i];
    }
}

// Initialize TimerA for 1 millisecond interrupt
void TCA0_init(void)
{
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;                              // Enable overflow interrupt
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;                      // Normal mode
    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);                         // Disable event counting
    TCA0.SINGLE.PER = (F_CPU / 1000000) - 1;                              // Set period for 1µs (F_CPU in Hz)
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm; // Start timer with no prescaler
}
// Start the timer
void start_timer(void)
{
    timer_count = 0;
    measure_active = 1;
}

// Stop the timer and capture pulse width
void stop_timer(void)
{
    measure_active = 0;
    pulse_width = timer_count;
}
// Interrupt Service Routine for TimerA overflow
ISR(TCA0_OVF_vect)
{
    if (measure_active)
    {
        timer_count++;
    }
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // Clear interrupt flag
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

// Read distance from the ultrasonic sensor
void read_distance(ultrasonic_sensor *sensor)
{   
    // USART2_PRINTF("Entered read_distance method.\r\n");
    const uint16_t timeout_us = 36000; 

    // Send a 10us pulse to the Trig pin
    PORTA.OUTCLR = sensor->TRIG_PIN;
    _delay_us(2);
    PORTA.OUTSET = sensor->TRIG_PIN;
    _delay_us(10);
    PORTA.OUTCLR = sensor->TRIG_PIN;

    // Timeout if echo never goes high
    start_timer(); 
    while (!(PORTA.IN & sensor->ECHO_PIN)) {
        if (timer_count >= timeout_us) {
            stop_timer();
            USART2_PRINTF("Echo pin did not go high - timeout occurred.\r\n");
            sensor->distance = -1; // Error in sensing 
            return;
        }
    }
    
    start_timer();
    while (PORTA.IN & sensor->ECHO_PIN); // Wait for Echo pin to go low
    stop_timer();

    // Calculate distance in cm
    float distance = ((float)pulse_width * 0.34) / 2.0;
    distance = clamp(distance, 0.0f, 30.0f); 

    // Update sensor with distance reading
    sensor->distance = distance;
}


float calculate_turn(float d1, float d2)
{
    // USART2_PRINTF("Calculating angle");
    // takes in 2 distances from sensors and computes degrees to turn
    float L = 3.0f; // l is distance between sensors
    // atanf returns in radians cringe jk  
    float radians = atanf((d1 - d2) / L); 
    float degrees = radians * (180.0f / M_PI);
    return clamp(degrees, -45.0f, 45.0f);
    
}

float calculate_forward_distance(float d1, float d2, float angle)
{
    // USART2_PRINTF("Calculating forward dist.\r\n");
    float angle_radians = angle * (M_PI / 180.0f); // Convert to radians
    // scale via cos
    float distance = ((d1 + d2) / 2.0f) * cosf(angle_radians); 
    // if the angle to turn is close to 0 we shouldn't more to much 
    distance -= 5.0f;
    return clamp(distance, 0.0f, 100.0f); // for now robot shouldnt need to travel more than 100 cms  
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

// delay in microseconds using the existing timer
void my_delay_us(uint16_t delay_us) {
    timer_count = 0;
    measure_active = 1;

    // Wait until the specified delay in microseconds is reached
    while (timer_count < delay_us);

    measure_active = 0; // Stop the measurement
}