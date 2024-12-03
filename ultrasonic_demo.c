#define F_CPU 3333333

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <string.h>


#define TRIG_PIN PIN3_bm    // A3 as Trig
#define ECHO_PIN PIN2_bm    // A2 as Echo

#define SAMPLES_PER_BIT 16
#define USART2_BAUD_VALUE(BAUD_RATE) (uint16_t)((F_CPU << 6) / (((float) SAMPLES_PER_BIT) * (BAUD_RATE)) + 0.5)


volatile uint16_t pulse_width = 0;
volatile uint16_t timer_count = 0;
volatile uint8_t measure_active = 0;

// USART functions
void USART2_INIT(void);
void USART2_PRINTF(char *str);

// Ultrasonic functions
void ultrasonic_init(void);
float read_distance(void);

// Timer functions
void TCA0_init(void);
void start_timer(void);
void stop_timer(void);

int main(void) {
    USART2_INIT();       // Initialize USART2
    ultrasonic_init();   // Initialize GPIO for ultrasonic sensor
    TCA0_init();       // Initialize TimerB for pulse width measurement

    char buffer[50];     // Buffer for storing distance output
    
    sei(); 

    while (1) {
        float distance = read_distance();  // Read distance

        // Format and send distance over USART
        snprintf(buffer, sizeof(buffer), "Distance: %.2f cm\r\n", distance);
        USART2_PRINTF(buffer);

        _delay_ms(100);  // Delay between readings
    }
}

// Initialize USART2
void USART2_INIT(void) {
    PORTF.DIRSET = PIN0_bm;      // Set TX (PF0) as output
    PORTF.DIRCLR = PIN1_bm;      // Set RX (PF1) as input
    USART2.BAUD = USART2_BAUD_VALUE(9600);   // Set baud rate to 9600
    USART2.CTRLB |= USART_TXEN_bm;  // Enable TX
}

// Send a string over USART2
void USART2_PRINTF(char *str) {
    for (size_t i = 0; i < strlen(str); i++) {
        while (!(USART2.STATUS & USART_DREIF_bm));   // Wait for the buffer to be ready
        USART2.TXDATAL = str[i];
    }
}

// Initialize GPIO for ultrasonic sensor
void ultrasonic_init(void) {
    PORTA.DIRSET = TRIG_PIN;  // Set Trig pin (A3) as output
    PORTA.DIRCLR = ECHO_PIN; // Set Echo pin (A2) as input
}


// Initialize TimerA for 1 millisecond interrupt
void TCA0_init(void) {
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;        // Enable overflow interrupt
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // Normal mode
    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);   // Disable event counting
    TCA0.SINGLE.PER = (F_CPU / 1000000) - 1;        // Set period for 1µs (F_CPU in Hz)
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm; // Start timer with no prescaler
}
// Start the timer
void start_timer(void) {
    timer_count = 0;
    measure_active = 1;
}

// Stop the timer and capture pulse width
void stop_timer(void) {
    measure_active = 0;
    pulse_width = timer_count;
}
// Interrupt Service Routine for TimerA overflow
ISR(TCA0_OVF_vect) {
    if (measure_active) {
        timer_count++;
    }
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // Clear interrupt flag
}

// Measure the pulse width on the Echo pin
uint16_t get_pulse_width(void) {
    TCB0.CNT = 0;                     // Clear counter
    while (!(TCB0.INTFLAGS & TCB_CAPT_bm));  // Wait for capture event
    TCB0.INTFLAGS = TCB_CAPT_bm;      // Clear capture flag
    return TCB0.CCMP;                 // Return captured value
}

// Read distance from the ultrasonic sensor
float read_distance(void) {
    // Send a 10us pulse to the Trig pin
    PORTA.OUTCLR = TRIG_PIN;
    _delay_us(2);
    PORTA.OUTSET = TRIG_PIN;
    _delay_us(10);
    PORTA.OUTCLR = TRIG_PIN;

    // Wait for Echo pin to go high (start timing)
    while (!(PORTA.IN & ECHO_PIN));
    start_timer();

    // Wait for Echo pin to go low (stop timing)
    while (PORTA.IN & ECHO_PIN);
    stop_timer();

    // Calculate distance in cm
    float distance = ((float)pulse_width * 0.34) / 2.0;
    return distance;
}