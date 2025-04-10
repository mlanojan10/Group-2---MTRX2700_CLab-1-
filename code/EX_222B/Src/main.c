//EX 2.2.2 B incorporates a "CALLBACK"
// This is a function that you pass as an argument to another piece of code,
//so that it can be called ("called back") later, when something happens - in this case the when the button is pressed

#include <stdint.h>
#include <stm32f303xc.h>

#include "button.h"   // include the button module
//#include "leds.h"     // you'll create this soon

void enable_clocks() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}

void initialise_board() {
	uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	*led_output_registers = 0x5555;
}

void chase_led() {
	static uint8_t led_mask = 0;
	static uint8_t direction = 1;
	uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;

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

	*led_register = led_mask;
}

void test_callback(void) {
    //this function is there for testing purposes. It shows that the callback function is working
	//It will first turn on the PE8 LED (which flashes off then on, and then turns on/off the LED as requested by the task
	//To implement testing, change the line in in the main function
	// Please note this function acts very slow - but it is to understand what is going on

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

int main(void)
{
	enable_clocks();
	initialise_board();

	// register chase_led as a button press callback
	//button_init(chase_led);

	//Testing Line (make sure to comment the above button_init function first
	button_init(test_callback);

	while (1) {
		// main loop can stay empty
	}
}
