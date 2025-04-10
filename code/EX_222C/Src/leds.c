#include "stm32f303xc.h"
#include "leds.h"

// LED GPIO Initialization (GPIOE, assuming LEDs are connected here)
void leds_init(void) {
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
