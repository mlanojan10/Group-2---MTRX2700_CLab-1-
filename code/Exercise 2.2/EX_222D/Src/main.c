//EX 2.2.2 Advanced Functionality is used to ensure spamming of the button is not recorded.
//It is chosen that the interval between allowed registered button presses is 1 Second
//This is mainly controlled in button.c file

#include <stdint.h>
#include <stm32f303xc.h>
#include "digitalio.h"   // include the button module
//#include "leds.h"     // include the new LED module


int main(void) {
    enable_clocks();
    initialise_board();

    // Register chase_led as a button press callback
    button_init(chase_led);

    while (1) {
        // main loop can stay empty
    }
}
