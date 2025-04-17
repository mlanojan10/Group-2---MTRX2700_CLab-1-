#include <stdint.h>
#include "stm32f303xc.h"
#include "timer.h"

// Enable the clocks for desired peripherals (GPIOA, C and E)
void timer_enable_clocks() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

// Initialise the discovery board I/O (just outputs: inputs are selected by default)
void timer_initialise_board() {

	// Get a pointer to the second half word of the MODER register (for outputs pe8-15)
	uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	*led_output_registers = 0x5555; // All LEDs are on
}

// General function called to make new prescaler value take effect
// Input: desired timer number
void trigger_prescaler(TIM_TypeDef *TIM) {

	TIM->ARR = 0x01;
	TIM->CNT = 0x00;
	asm("NOP");
	asm("NOP");
	asm("NOP");
	TIM->ARR = 0xffffffff;
}

// Store a pointer to the function that is called when a timer interrupt occurs
void (*on_timer_interrupt)() = 0x00;


// Initialise timer with 1ms ticks to trigger a callback function regularly
// Input: desired timer number to initialise; callback function
void init_timer_module(TIM_TypeDef *TIM, void (*timer_callback)()) {

	TIM->CR1 |= TIM_CR1_CEN;
	TIM->PSC = 7999;
	trigger_prescaler(TIM);
	TIM->CR1 &= ~TIM_CR1_CEN;
	TIM->CNT = 0;

	on_timer_interrupt = timer_callback;
}

// Simple function to switch on/off every led when called
void blink_all_leds() {
	uint8_t *led_output_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
	*led_output_register ^= 0b11111111;
}

// Simple function to switch on/off every second led when called
void blink_alternate_leds() {
	uint8_t *led_output_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
	*led_output_register ^= 0b10101010;
}

// Interrupt Service Routine
void TIM2_IRQHandler(void) {
    // Check if the TIM2 interrupt flag is set
    if (TIM2->SR & TIM_SR_UIF) {
		// Run the callback function (make sure it is not null first)
        if (on_timer_interrupt != 0x00) {
        	on_timer_interrupt();
        	// If timer is in one-pulse mode reset timer to default mode
			if (TIM2->SR & TIM_SR_CC1IF) {
				// Disable capture/compare flag
				TIM2->SR &= ~TIM_SR_CC1IF;
        		// Disable the Capture/Compare 1 interrupt
				TIM2->DIER &= ~TIM_DIER_CC1IE;
			}
        }
        // Clear the interrupt flag (write 1 to the UIF bit to reset it)
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

// Enable hardware interrupt for timer 2
void enable_timer2_interrupt() {
	// Disable the interrupts while messing around with the settings
	// Otherwise can lead to strange behaviour
	__disable_irq();

	// Enable update interrupt (UIE)
	TIM2->DIER |= TIM_DIER_UIE;

	// Tell the NVIC module that TIM2 interrupts should be handled
	NVIC_SetPriority(TIM2_IRQn, 1);  // Set Priority
	NVIC_EnableIRQ(TIM2_IRQn);

	// Re-enable all interrupts (now that we are finished)
	__enable_irq();
}

// Simple get function to return a specific timer's period in ms
// Assuming timers are configured to 1kHz so each count is 1ms
uint32_t get_timer_period(TIM_TypeDef *TIM) {
	return TIM->ARR;
}

// Simple set function to set a specific timer's period in ms
// Assuming timers are configured to 1kHz so each count is 1ms
void set_timer_period(TIM_TypeDef *TIM, uint32_t new_period) {
	TIM->ARR = new_period;
}

// Function to reset a specific timer's count with a new period in ms
// Assuming timers are configured to 1kHz so each count is ms
void reset_timer(TIM_TypeDef *TIM, uint32_t new_period) {

	// Stop timer's count and reset count to 0
	TIM->CR1 &= ~TIM_CR1_CEN;
	TIM->CNT = 0;
	// Set new period
	set_timer_period(TIM, new_period);
	// Restart timer
	TIM->CR1 |= TIM_CR1_CEN;
}

// Hardware interrupt enable for converting a specific timer's operation to one-pulse mode
// Input specific timer to be used and delay desired in ms
void one_shot(TIM_TypeDef *TIM, uint32_t delay) {

	// Sets value for capture/compare event
	TIM->CCR1 = delay;

	// Enable one-pulse mode
	TIM->CR1 |= TIM_CR1_OPM;

	// Enable capture/compare interrupt
	TIM->DIER |= TIM_DIER_CC1IE;

	reset_timer(TIM, delay);
}
