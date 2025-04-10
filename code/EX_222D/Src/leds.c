#include "stm32f303xc.h"
#include "leds.h"

// LED GPIO Initialization (GPIOE, assuming LEDs are connected here)
void leds_init(void) {
	// Get a pointer to the second half word of the MODER register (for outputs PE8-PE15)
	    uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	    *led_output_registers = 0x5555;  // Set PE8-PE15 as output (01)
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
