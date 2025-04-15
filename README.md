# Group-2---MTRX2700_CLab-1-
## Digital IO 
#### Digital IO manages the operation of gaining an input and displaying a visual output using LED’s on the microcontroller. This functionality is controlled using interrupts within a main function to ensure responsive, asynchronous handling of user input without constantly polling the button in the main loop.. All code begins with enabling the necessary GPIO clocks and configuring the LEDs as outputs.

### Part A 
The button is connected to pin PA0 and is congifgured to generate the interrupt on a rising edge - meaning the interrupt is triggered when the button is pressed. This is done through the enable_interrupt() function, which enables the required SYSCFG clock and selects PA0 as the input source by connecting PA0 to EXTI line 0, so the microcontroller knows which pin to watch: 
```
void enable_interrupt() {
    __disable_irq();  // Disable global interrupts during setup

    // Enable SYSCFG clock (needed to configure external interrupts)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Map EXTI line 0 to PA0
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;

    // Configure EXTI line 0 to trigger on rising edge (button press)
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // Unmask EXTI line 0 interrupt (enable it)
    EXTI->IMR |= EXTI_IMR_MR0;

    // Set interrupt priority and enable EXTI0 interrupt in NVIC
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);

    __enable_irq();  // Re-enable global interrupts
}
```
It also configured the Nested Vevotred Interrupt Controller to set priority to the interrupt and enable its function. 

When the button is pressed, the hardware triggers EXTI line 0, causing the EXTI0_IRQHandler() interrupt service routine (ISR) to execute. Inside this ISR, the program first checks if the function pointer on_button_press is non-null.
```
void EXTI0_IRQHandler(void)
{
	// run the button press handler (make sure it is not null first !)
	if (on_button_press != 0x00) {
		on_button_press();
	}

	// reset the interrupt (so it doesn't keep firing until the next trigger)
	EXTI->PR |= EXTI_PR_PR0;
}
```
If it is, it calls the function chase_led(), that controls the state of the LEDs in a "chase" pattern, turning LED’s on and then off progressively, depending on the current LED state.  After calling the handler function, the ISR clears the interrupt allowing the system to detect the next button press.

### Part B 
Previously, it is decided what happens when a button is pressed by writing code inside the interrupt handler (EXTI0_IRQHandler), which is not flexible for the extension of different projects (you will need to change the code in the interrupt handler every time). A callback is used in this part to pass a function as a pointer to another part of the code to be later used and improves the flexibility of the code.

The code is now split into different files, consisting of digitalio.c and main.c. Within digitalio.c, the variable button_callback_t is used to store the address of a function to be runned when the button is pressed: 
```
static button_callback_t on_button_press = 0x00;
```
Then, in button_init(callback), a function pointer is passed (in this case chase_led), and stored, hence on_button_press will hold the functions address:
```
on_button_press = callback;
```

When the button is pressed, the microcontroller triggers EXTI0_IRQHandler() and calls the function through the pointer without changing the code of the interrupt handler. 
```
if (on_button_press != 0x00) {
    on_button_press();
}
```
Within the main.c file, the function to be used a is defined, in this case chase_led():
```
button_init(chase_led);
}
```



### Part C 
This part further improves modularity of the code by using get/set functions for LED control, so the LED states can only be accessed through leds_get_state() and leds_set_state() hiding direct GPIO access inside the digitalio.c module. This is controlled by 3 main functions:
leds_init():
Enables the clock for GPIOE.
Configures PE8–PE15 as outputs.

leds_get_state():
Reads the current 8-bit state of PE8–PE15.
Returns this as an 8-bit value.

leds_set_state():
Updates the state of PE8–PE15 based on an 8-bit bitmask.
Leaves PE0–PE7 unchanged.

Using these functions provides a more modular and safe interface.

### Part D 
To ensure the efficiency of the program further, this part incorporates button control and limits the ability for the user to “spam” the button. 

This is done by using Timer 2 (TIM2) and its corresponding flags to manage a cooldown period of 1 second between button presses. 
```
TIM2->PSC = 7999;
TIM2->ARR = 1000;
```
f = (8 Mhz)/(7999+1) = 8000000/8000 = 1000 Hz 

Time to overflow = ARR x time per tick 
Time to overflow = 1000 x 1/1000 = 1 second 


When the timer overflows after this time, it triggers a timer interrupt that sets the flag (button_ready), which will then allow the LED function (chase_led()) to run. 
```
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
```
Once running, it clears the timer flag and resets the timer, disabling further button presses until the button_ready flag is ready. 



### Testing 


## Serial Interface 

### Testing 

## Timer Interface
### Timer 
### One Shot 
### Testing 

## Integration 
![Image](https://github.com/user-attachments/assets/0ebd02c1-add6-4da5-9c6d-872a68a26e80)

### Testing 
