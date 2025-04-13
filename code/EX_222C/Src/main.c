//EX 2.2.2 C is an improved modularised code using Get and Set functions
//Modular Implementation is controlled by led.h function, please go to those files for explanation
//Instead of directly manipulating GPIOE->ODR in chase_led, the code now uses leds_get_state()
//to read the LED state and leds_set_state() to update it.

#include <stdint.h>
#include <stm32f303xc.h>
#include "digitalio.h"   // include the button module

void enable_clocks() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}

void initialise_board() {
    // Initialize the LEDs using the new LED module
    leds_init();
}

void chase_led(void) {
    static uint8_t led_mask = 0;
    static uint8_t direction = 1;

    if (direction) {
        led_mask = (led_mask << 1) | 1;
        if (led_mask == 0xFF) {
            direction = 0;
        }
    } else {
        led_mask >>= 1;
        if (led_mask == 0x00) {
            direction = 1;
        }
    }

    leds_set_state(led_mask);
}


void test_callback(void) {
    // This function is for testing purposes only and should be used to test the callback functionality.
    // For now, it just turns on PE8, waits a bit, and then turns it off.
    // Afterwards, it starts the chase LED function.

    // Turn on PE8 (LED)
    GPIOE->ODR |= (1 << 8);

    // Simple software delay (you can adjust the delay loop to make it more visible)
    for (volatile int i = 0; i < 1000000; i++) {}  // Increase delay time for visibility

    // Turn off PE8 (LED)
    GPIOE->ODR &= ~(1 << 8);

    // Simple software delay to ensure the LED stays off for a short period
    for (volatile int i = 0; i < 1000000; i++) {}  // Adjust delay time here

    // Then call the chase_led function
    chase_led();
}

int main(void) {
    enable_clocks();
    initialise_board();

    // Register chase_led as a button press callback
    button_init(chase_led);

    while (1) {
        // main loop can stay empty
    }
}
