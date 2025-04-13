#include <stm32f303xc.h>
#include "digitalio.h"

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
