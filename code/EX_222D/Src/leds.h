//This file along with Led.c essentially has three main functions:

#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

// Function to initialize LEDs (GPIO configuration)
void leds_init(void);

// Function to get the current LED state (as a bitmask)
//Reads the ODR (Output Data Register) and returns the LED state as a 8 bit mask
//This is called in thechas_led function that modifies the bitmask as required
uint8_t leds_get_state(void);

// Function to set the LED state (as a bitmask)
//Updates the LEDs after chase_led has finished modifying it
void leds_set_state(uint8_t state);

#endif // LEDS_H
