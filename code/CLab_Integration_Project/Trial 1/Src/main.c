//EX 2.2.2 C is an improved modularised code using Get and Set functions
//Modular Implementation is controlled by led.h function, please go to those files for explanation
//Instead of directly manipulating GPIOE->ODR in chase_led, the code now uses leds_get_state()
//to read the LED state and leds_set_state() to update it.

#include <stdint.h>
#include <stm32f303xc.h>
#include "digitalio.h"   // include the button module

uint8_t pattern_buffer[] = "11011001";  // ASCII string, not binary literal
//char command_buffer[]="led"

int main(void) {
    enable_clocks();
    initialise_board();

    //button_init(chase_led);   //original chase led with button exercis


    //These functions are used to display the pattern on the LED's - can be put under if else statement for LED's
    pattern_callback_init(display_pattern_callback);

    trigger_pattern_display(pattern_buffer);

    while (1) {
        // main loop can stay empty
    }
}
