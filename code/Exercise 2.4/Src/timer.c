#include <stdint.h>
#include "stm32f303xc.h"
#include "timer.h"

// status of timer operation, 0 = regular interval mode, 1 = one-shot mode
uint8_t timer_mode = 0;

// enable the clocks for desired peripherals (GPIOA, C and E)
void enable_clocks() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

// initialise the discovery board I/O (just outputs: inputs are selected by default)
void initialise_board() {
	// get a pointer to the second half word of the MODER register (for outputs pe8-15)
	uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	*led_output_registers = 0x5555; //every second led is on
}

// general function called to make new prescaler value take effect
// input: desired timer number
void trigger_prescaler(TIM_TypeDef *TIM) {

	TIM->ARR = 0x01;
	TIM->CNT = 0x00;
	asm("NOP");
	asm("NOP");
	asm("NOP");
	TIM->ARR = 0xffffffff;
}

// store a pointer to the function that is called when a timer interrupt occurs
void (*on_timer_interrupt)() = 0x00;


// initialise timer with delay time in ms to trigger a callback function regularly
// input: desired timer number to initialise; delay time in ms; callback function
void init_timer_module(TIM_TypeDef *TIM, uint32_t interval, void (*timer_callback)()) {

	TIM->CR1 |= TIM_CR1_CEN;
	TIM->PSC = 7999;
	trigger_prescaler(TIM);
	TIM->CR1 = 0;
	TIM->CNT = 0;
	TIM->ARR = interval;
	TIM->CR1 |= TIM_CR1_CEN;

	on_timer_interrupt = timer_callback;

}

// simple function to switch on/off every second led when called
void change_pattern() {

	uint8_t *led_output_register = ((uint8_t*)&(GPIOE->ODR)) + 1;
	*led_output_register ^= 0b10101010;
}

// function called when timer interrupt is raised
void TIM2_IRQHandler(void) {
    // Check if the TIM2 interrupt flag is set
    if (TIM2->SR & TIM_SR_UIF) {
        // run the callback function (make sure it is not null first)
        if (on_timer_interrupt != 0x00) {

        	on_timer_interrupt();

        	// if timer is in one-shot mode then stop timer and reset timer to default mode
			if (timer_mode == 1) {
        		TIM2->CR1 = 0;
        		timer_mode = 0;
			}

        }
        // Clear the interrupt flag (write 1 to the UIF bit to reset it)
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

// intialise hardware interrupt for timer 2
void enable_timer2_interrupt() {
	// Disable the interrupts while messing around with the settings
	//  otherwise can lead to strange behaviour
	__disable_irq();

	// Enable update interrupt (UIE)
	TIM2->DIER |= TIM_DIER_UIE;

	// Tell the NVIC module that EXTI0 interrupts should be handled
	NVIC_SetPriority(TIM2_IRQn, 1);  // Set Priority
	NVIC_EnableIRQ(TIM2_IRQn);

	// Re-enable all interrupts (now that we are finished)
	__enable_irq();
}

// simple get function to return a specific timer's period in ms
// assuming timers are configured to 1kHz so each count is 1ms
uint32_t get_timer_period(TIM_TypeDef *TIM) {
	return TIM->ARR;
}

// simple set function to set a specific timer's period in ms
// assuming timers are configured to 1kHz so each count is 1ms
void set_timer_period(TIM_TypeDef *TIM, uint32_t new_period) {
	TIM->ARR = new_period;
}

// function to reset a specific timer's count with a new period in ms
// assuming timers are configured to 1kHz so each count is ms
void reset_timer(TIM_TypeDef *TIM, uint32_t new_period) {

	// stop timer's count and reset count to 0
	TIM->CR1 = 0;
	TIM->CNT = 0;
	// set new period
	set_timer_period(TIM, new_period);
	// restart timer
	TIM->CR1 |= TIM_CR1_CEN;
}

// software implementation to convert a specific timer's operation to one-shot mode by using a flag
// input delay desired in ms
void one_shot(TIM_TypeDef *TIM, uint32_t delay) {

	timer_mode = 1;

	// only updates the timer's period when needed
	if (get_timer_period(TIM) != delay) {
		reset_timer(TIM, delay);
	}
}
