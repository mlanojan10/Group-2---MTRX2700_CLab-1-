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
#### LED Interrupt Testing 
Since there is an expected LED behaviour, the chase_led function was tested through visual analysis ensuring functionality through these steps:
Press the button and observe bahaviour of LED 
Behaviour should follow expected: 
| Button Press | Binary Representation | Button Press | Binary Representation |
|:------------|:---------------------|:------------|:---------------------|
| 0            | 00000000               | 9            | 01111111               |
| 1            | 00000001               | 10           | 00111111               |
| 2            | 00000011               | 11           | 00011111               |
| 3            | 00000111               | 12           | 00001111               |
| 4            | 00001111               | 13           | 00000111               |
| 5            | 00011111               | 14           | 00000011               |
| 6            | 00111111               | 15           | 00000001               |
| 7            | 01111111               | 16           | 00000000               |
| 8            | 11111111               |               |                         |


#### Callback Functionality Testing 
To ensure callback functionality was working, the function was tested visually through the use of LED’s. The function test_callback(): 
```
void test_callback(void) {
    // Turn on PE8
    GPIOE->ODR |= (1 << 8);

    // Delay
    for (volatile int i = 0; i < 1000000; i++) {}

    // Turn off PE8
    GPIOE->ODR &= ~(1 << 8);

    // Delay
    for (volatile int i = 0; i < 1000000; i++) {}

    // Then call chase_led
    chase_led();
}
```
To test callback functionality, this function should be set as the function pointer in main: 
```
button_init(test_callback);
```
Once the button is pressed, the callback is determined as working if the PE8 LED flashes on with a small delay then continues with the chase_led functionality. This means that th callback function has registered and is working as required. 

#### Timer Functionality Testing 
The timer was tested by observing a unchanged LED pattern when spamming the button, until 1 second from the last observed LED toggle on. Using a stopwatch to verify the time intervals was used additionally. 




## Serial Interface 

### Testing 

## Timer Interface
### Timer 
### One Shot 
### Testing 

## Integration 
The integration tasks encapsulated all 3 modules together to provide a working user interface that can control each function and certain parameters through input into the laptop. All modules have stayed the same as what was explained above, however, there have been some slight changes to ensure proper functionality of the task.
![Image](https://github.com/user-attachments/assets/78ce1bd6-12cd-4ee3-96a9-8ea3d0a4fc39)
This flowchart depicts the integration task and key compoennts in its functionality.  

The main function handles (I am not bothered writing this, someone please do it): 
- Talk about the if - else statement and how it checks if the input is valid 
- How the buffer is split into commands and instructions and passed as an argument -double buffers? 
- How the struct within the main function handles overriding the mode so we get the proper function when a new user request is sent. 


#### Digital IO 
The integration task requires a certain user input bitmask to be displayed on the microcontroller LEDs, which is different from the previous parts needed in the digital IO interface. There is no longer a need for a button to control this and hence the function display_pattern_callback() was added to the module: 
```
void display_pattern_callback(uint8_t *buffer) {
	// Convert ASCII binary string (e.g., "11011110") to uint8_t pattern
	uint8_t pattern = 0;

	for (int i = 0; i < 8; i++) {
		if (buffer[i] == '1') {
			pattern |= (1 << (7 - i));  // MSB first
		} else if (buffer[i] != '0') {
			// Invalid character found — stop and do not update LEDs
			return;
		}
	}

	leds_set_state(pattern);
}
```
As the user input is given in ascii format, this function converts this to a binary 8 bit string and then sets the led pattern by the led_set_state() function. The function also ensures that only 8 bits are registered as the pattern, and further limits any issues of receiving incorrect input from the user.

### Testing 
<table>
  <tr>
    <th>Module</th>
    <th>User Input</th>
    <th>Expected</th>
  </tr>
  
  <!-- Digital IO -->
  <tr>
    <td rowspan="6"><b>Digital IO</b></td>
    <td>led 11111111</td>
    <td>All LEDs turn on</td>
  </tr>
  <tr>
    <td>led 00000000</td>
    <td>All LEDs remain off</td>
  </tr>
  <tr>
    <td>led 10101010</td>
    <td>Alternating pattern on LEDs</td>
  </tr>
  <tr>
    <td>led 01101010</td>
    <td>Corresponding LED pattern should appear</td>
  </tr>
  <tr>
    <td>led 111000111000101</td>
    <td>Only the newest 8 bits should display the pattern. In this case: 11100011</td>
  </tr>
  <tr>
    <td>led 11122111</td>
    <td>All LEDs remain off</td>
  </tr>

  <!-- Serial -->
  <tr>
    <td rowspan="2"><b>Serial</b></td>
    <td>serial hello</td>
    <td>“String: hello” should be returned</td>
  </tr>
  <tr>
    <td>serial MTRX2700 Project 2</td>
    <td>“String: MTRX2700 Project 2” should be returned</td>
  </tr>

  <!-- Timer -->
  <tr>
    <td rowspan="2"><b>Timer</b></td>
    <td>timer 1000</td>
    <td>Timer should have a period of 1 second for every flash on and off of pattern</td>
  </tr>
  <tr>
    <td>timer 3000</td>
    <td>Timer should have a period of 3 seconds for every flash on and off of pattern</td>
  </tr>

  <!-- One Shot -->
  <tr>
    <td rowspan="2"><b>One Shot</b></td>
    <td>oneshot 1000</td>
    <td>Pattern should stay on for 1 second then turn off forever</td>
  </tr>
  <tr>
    <td>oneshot 3000</td>
    <td>Pattern should stay on for 3 seconds then turn off forever</td>
  </tr>

  <!-- Other Input -->
  <tr>
    <td rowspan="2"><b>Other Input</b></td>
    <td>hello 1000</td>
    <td>“Unknown Command”</td>
  </tr>
  <tr>
    <td>timer1000</td>
    <td>“Invalid input format”</td>
  </tr>
</table>

| Module     | User Input                  | Expected Output                                                   |
|:------------|:----------------------------|:------------------------------------------------------------------|
| **Digital IO** |||
|             | `led 11111111`              | All LEDs turn on                                                  |
|             | `led 00000000`              | All LEDs remain off                                               |
|             | `led 10101010`              | Alternating pattern on LEDs                                       |
|             | `led 01101010`              | Corresponding LED pattern should appear                           |
|             | `led 111000111000101`       | Only the newest 8 bits should display. In this case: `11100011`   |
|             | `led 11122111`              | All LEDs remain off (invalid input ignored)                       |
| **Serial** |||
|             | `serial hello`              | `String: hello`                                                   |
|             | `serial MTRX2700 Project 2` | `String: MTRX2700 Project 2`                                      |
| **Timer** |||
|             | `timer 1000`                | Timer should have a period of 1 second for every flash on/off     |
|             | `timer 3000`                | Timer should have a period of 3 seconds for every flash on/off    |
| **One Shot** |||
|             | `oneshot 1000`              | Pattern should stay on for 1 second, then turn off forever        |
|             | `oneshot 3000`              | Pattern should stay on for 3 seconds, then turn off forever       |
