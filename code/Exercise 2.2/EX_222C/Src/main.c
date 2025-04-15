//EX 2.2.2 C is an improved modularised code using Get and Set functions
//Modular Implementation is controlled by led.h function, please go to those files for explanation
//Instead of directly manipulating GPIOE->ODR in chase_led, the code now uses leds_get_state()
//to read the LED state and leds_set_state() to update it.

#include <stdint.h>
#include <stm32f303xc.h>
#include "digitalio.h"   // include the button module

int main(void) {
    enable_clocks();
    initialise_board();

    // Register chase_led as a button press callback
    button_init(chase_led);


    while (1) {
        // main loop can stay empty
    }
}
