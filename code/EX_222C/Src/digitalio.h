#ifndef DIGITALIO_H
#define DIGITALIO_H

#include <stdint.h>

//-------------------------BUTTON CONTROL -----------------------

// Function pointer type for button press callbacks
typedef void (*button_callback_t)(void);

// Call this to initialize the button interrupt, pass in your callback
void button_init(button_callback_t callback);


//-------------------------LED CONTROL -----------------------

// Function to initialize LEDs (GPIO configuration)
void leds_init(void);

// Function to get the current LED state (as a bitmask)
//Reads the ODR (Output Data Register) and returns the LED state as a 8 bit mask
//This is called in thechas_led function that modifies the bitmask as required
uint8_t leds_get_state(void);

// Function to set the LED state (as a bitmask)
//Updates the LEDs after chase_led has finished modifying it
void leds_set_state(uint8_t state);

#endif // DIGITALIO_H
