# Group-2---MTRX2700_CLab-1-
## Digital IO 
 Digital IO manages the operation of gaining an input and displaying a visual output using LED’s on the microcontroller. This functionality is controlled using interrupts within a main function to ensure responsive, asynchronous handling of user input without constantly polling the button in the main loop.. All code begins with enabling the necessary GPIO clocks and configuring the LEDs as outputs.

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
If it is, it calls fucntion chase_led(), that controlls the state of the LEDs in a "chase" pattern, turning LED’s on and then off progressively, depending on the current LED state.  After calling the handler function, the ISR clears the interrupt allowing the system to detect the next button press.

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
