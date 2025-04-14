#ifndef DIGITALIO_H
#define DIGITALIO_H

#include <stdint.h>


//-------------------------MAIN CONTROL -----------------------

void enable_clocks();


void chase_led(void);


void test_callback(void);


void display_pattern_callback(uint8_t *buffer);

//---------------------PATTERN CONTROL---------------------------

// Callback typedef for pattern display
typedef void (*pattern_callback_t)(uint8_t *pattern_buffer);

// Register a callback for pattern display
void pattern_callback_init(pattern_callback_t callback);

// Trigger the registered pattern display callback with a buffer
void trigger_pattern_display(uint8_t *pattern_buffer);



//-------------------------BUTTON CONTROL -----------------------

// Function pointer type for button press callbacks
typedef void (*button_callback_t)(void);

// Call this to initialize the button interrupt, pass in your callback
void button_init(button_callback_t callback);


//-------------------------LED CONTROL -----------------------

// Function to initialize LEDs (GPIO configuration)
void initialise_board(void);

// Function to get the current LED state (as a bitmask)
//Reads the ODR (Output Data Register) and returns the LED state as a 8 bit mask
//This is called in thechas_led function that modifies the bitmask as required
uint8_t leds_get_state(void);

// Function to set the LED state (as a bitmask)
//Updates the LEDs after chase_led has finished modifying it
void leds_set_state(uint8_t state);

#endif // DIGITALIO_H
