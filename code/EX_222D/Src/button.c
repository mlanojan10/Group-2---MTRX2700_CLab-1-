#include "button.h"
#include <stm32f303xc.h>

// Static variable to store the button callback
static button_callback_t on_button_press = 0x00;

// Flag to track if the button is ready for another press (1 second cooldown)
volatile uint8_t button_ready = 1;

// Timer configuration to track cooldown
void configure_timer(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set prescaler (PSC) to divide the timer clock to get 1-second delay
    TIM2->PSC = 7999;  // Assuming the clock is 8 MHz (adjust according to your MCU's clock)
    TIM2->ARR = 1000;   // Set auto-reload to 1000 for 1 second delay

    // Enable the timer interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // Start the timer
    TIM2->CR1 |= TIM_CR1_CEN;

    // Enable NVIC interrupt for TIM2
    NVIC_EnableIRQ(TIM2_IRQn);
}

// Timer 2 interrupt handler (for cooldown)
void TIM2_IRQHandler(void) {
    // Clear the interrupt flag by writing 1 to UIF (Update Interrupt Flag)
    TIM2->SR &= ~TIM_SR_UIF;

    // Re-enable button presses
    button_ready = 1;
}

// Button press interrupt handler
void EXTI0_IRQHandler(void) {
    // Only handle button press if it's ready (i.e., cooldown has passed)
    if (button_ready && on_button_press != 0x00) {
        // Call the callback function
        on_button_press();

        // Disable further button presses during cooldown
        button_ready = 0;

        // Start the timer for cooldown
        TIM2->CNT = 0;  // Reset the timer counter to start the cooldown
    }

    // Clear the interrupt flag
    EXTI->PR |= EXTI_PR_PR0;
}

void button_init(button_callback_t callback) {
    on_button_press = callback;

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PA0 as input (default after reset, so can be skipped technically)

    // Map PA0 to EXTI0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // 0x00 = PA0

    // Set rising edge trigger
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // Unmask EXTI0 (enable interrupt for PA0)
    EXTI->IMR |= EXTI_IMR_MR0;

    // Set NVIC priority and enable EXTI0 interrupt
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);

    // Configure the timer for cooldown
    configure_timer();
}
