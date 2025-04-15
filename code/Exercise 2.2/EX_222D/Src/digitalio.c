 #include "digitalio.h"
#include <stm32f303xc.h>



//------------------------------MAIN CONTROL---------------------------------------------
void enable_clocks() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;
}

void initialise_board() {
    // Initialize the LEDs using the new LED module
    leds_init();
}

void chase_led(void) {
    static uint8_t led_mask = 0;
    static uint8_t direction = 1;

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

    leds_set_state(led_mask);
}


void test_callback(void) {
    // This function is for testing purposes only and should be used to test the callback functionality.
    // For now, it just turns on PE8, waits a bit, and then turns it off.
    // Afterwards, it starts the chase LED function.

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

//-----------------------BUTTON CONTROL------------------------------------------

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

//-------------------------LED CONTROL-----------------------------------------
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

