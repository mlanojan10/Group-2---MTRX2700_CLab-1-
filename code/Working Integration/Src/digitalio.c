#include "digitalio.h"
#include <stm32f303xc.h>


//-------------------------MAIN CONTROL -----------------------

//Enable clocks for the STM
void enable_clocks() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}

//This function handles the chasing effect of the LED's. It will be used when there is a button interrupt
void chase_led(void) {
    static uint8_t led_mask = 0;        // Holds the current LED pattern
    static uint8_t direction = 1;       // 1 for left-shift, 0 for right-shift this controls direction of turning on and off LEDs

    if (direction) {
        led_mask = (led_mask << 1) | 1; // Shift the LED pattern one bit to the left and turn on the new bit (OR with 1)
        if (led_mask == 0xFF) {
            direction = 0;              // If all LEDs are on change direction
        }
    } else {
        led_mask >>= 1;                // Shift the LED pattern one bit to the right, start turning off
        if (led_mask == 0x00) {
            direction = 1;             //When all LEDs are off, switch direction to start turning them on again
        }
    }

    leds_set_state(led_mask);  //Set new pattern
}


void test_callback(void) {
    // This function is for testing purposes only and should be used to test the callback functionality.
    // For now, it just turns on PE8, waits a bit, and then turns it off.
    // Afterwards, it starts the chase LED function.

    // Turn on PE8 (LED)
    GPIOE->ODR |= (1 << 8);

    // Simple software delay to show change
    for (volatile int i = 0; i < 1000000; i++) {}  // Increase delay time for visibility

    // Turn off PE8 (LED)
    GPIOE->ODR &= ~(1 << 8);

    // Sotware delay to ensure the LED stays off for a short period
    for (volatile int i = 0; i < 1000000; i++) {}  // Adjust delay time here

    chase_led();    //Call normal chase function
}

void display_pattern_callback(uint8_t *buffer) {
	// Convert ASCII binary string (e.g., "11011110") to uint8_t pattern
	uint8_t pattern = 0;

	for (int i = 0; i < 8; i++) {    // Loop through ONLY 8 characters in the buffer
		if (buffer[i] == '1') {      // If the character is '1', set the corresponding bit in 'pattern'
			pattern |= (1 << (7 - i));  // Ensures the first character goes to the highest bit
		} else if (buffer[i] != '0') {
			// Invalid character found — stop and do not update LEDs
			return;
		}
	}

	leds_set_state(pattern);      //pass the new pattern to LEDs
}

//-----------------------PATTERN CONTROL------------------------

// Define a static (file-scope) variable to store the callback function pointer
static pattern_callback_t pattern_display_callback = 0x00;

// Initialize the pattern display callback function
void pattern_callback_init(pattern_callback_t callback) {
    pattern_display_callback = callback;    // Store the provided callback function in the static variable

}

void trigger_pattern_display(uint8_t *pattern_buffer) {
    if (pattern_display_callback != 0x00) {       // Check if a valid callback has been registered
        pattern_display_callback(pattern_buffer); // Call the registered callback function with the provided pattern buffer

    }
}

void DigitalIO_SetPattern(char *pattern) {
	 	 // Call trigger_pattern_display, casting the string to a uint8_t pointer
	 // This passes the ASCII pattern string to the callback function
    trigger_pattern_display((uint8_t*)pattern);
}


//-------------------------BUTTON CONTROL -----------------------

// Static variable only accessible in this file
static button_callback_t on_button_press = 0x00;

void EXTI0_IRQHandler(void)
{
	if (on_button_press != 0x00) {
		on_button_press();
	}

	EXTI->PR |= EXTI_PR_PR0; // clear interrupt flag
}

void button_init(button_callback_t callback)
{
	on_button_press = callback;

	// Enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Configure PA0 as input (default after reset, so can be skipped technically)

	// Map PA0 to EXTI0
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // 0x00 = PA0

	// Set rising edge trigger
	EXTI->RTSR |= EXTI_RTSR_TR0;

	// Unmask EXTI0
	EXTI->IMR |= EXTI_IMR_MR0;

	// Set NVIC priority and enable EXTI0 interrupt
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);
}


//-------------------------LED CONTROL -----------------------

// LED GPIO Initialization (GPIOE, assuming LEDs are connected here)
void initialise_board(void) {
    // Enable clock for GPIOE
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

    // Set pins PE8-PE15 as output
    GPIOE->MODER &= ~(0xFFFF0000);  // Clear mode bits for PE8-PE15
    GPIOE->MODER |= 0x55550000;     // Set PE8-PE15 to output mode (01)
}

// Get the current state of the LEDs (PE8 to PE15 as a bitmask)
uint8_t leds_get_state(void) {
    uint8_t state = 0;
    state = (GPIOE->ODR >> 8) & 0xFF;  // Read ODR for PE8-PE15 and mask lower 8 bits
    return state;
}

// Set the state of the LEDs (PE8 to PE15 using a bitmask)
void leds_set_state(uint8_t state) {
    GPIOE->ODR = (GPIOE->ODR & 0x00FF) | (state << 8); // Set only PE8-PE15
}
