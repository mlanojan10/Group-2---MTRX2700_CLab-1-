#include "digitalio.h"
#include <stm32f303xc.h>


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
