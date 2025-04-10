//EX 2.2.2 Advanced Functionality is used to ensure spamming of the button is not recorded.
//It is chosen that the interval between allowed registered button presses is 1 Second
//This is mainly controlled in button.c file

#include <stdint.h>
#include <stm32f303xc.h>
#include "button.h"   // include the button module
#include "leds.h"     // include the new LED module

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

    // Get the current state of the LEDs
    led_mask = leds_get_state();

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

    // Set the new state of the LEDs
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
