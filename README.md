# Group-2---MTRX2700_CLab-1-

## Details about the project
### Group Member Roles & Responsibilities:

- **Sharon Britto**  
  Worked on Exercise 1 and Exercise 4. Responsible for writing code and user instructions.

- **Luc Do**  
  Worked on Exercise 2 and Exercise 4. Wrote code and user instructions for parts **b** and **d**.

- **Melvin Lanojan**  
  Worked on Exercise 2 and Exercise 4. Wrote code and user instructions for parts **a** and **c**. Also responsible for recording meeting minutes.

- **Johnny Wang**  
  Worked on Exercise 3 and Exercise 4. Responsible for writing code and user instructions.


## Digital IO 
Digital IO manages the operation of gaining an input and displaying a visual output using LED’s on the microcontroller. This functionality is controlled using interrupts within a main function to ensure responsive, asynchronous handling of user input without constantly polling the button in the main loop.. All code begins with enabling the necessary GPIO clocks and configuring the LEDs as outputs.

### Part A 
The button is connected to pin PA0 and is configured to generate the interrupt on a rising edge - meaning the interrupt is triggered when the button is pressed. This is done through the enable_interrupt() function, which enables the required SYSCFG clock and selects PA0 as the input source by connecting PA0 to EXTI line 0, so the microcontroller knows which pin to watch: 
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
- Reads the current 8-bit state of PE8–PE15.
- Returns this as an 8-bit value.

leds_set_state():
- Updates the state of PE8–PE15 based on an 8-bit bitmask.
- Leaves PE0–PE7 unchanged.

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
The Serial Interface module manages communication between the microcontroller and external devices using UART. It receives and transmits data asynchronously through the use of interrupts, allowing the system to handle serial communication efficiently without polling. The module buffers incoming characters until a terminating character is received, at which point a callback function is triggered to process the complete message. Double buffering is implemented to allow simultaneous receiving and processing of data, ensuring smooth and reliable operation even under continuous data flow.
### Part A 
This part implements a blocking input method for capturing user input via UART. Each character is polled and stored until Enter is pressed, after which the entire string is echoed.

#### Usage

When the program runs, the user types a string into the serial terminal. Pressing Enter triggers the system to echo the received line and await the next input.

#### Valid input

Any sequence of characters terminated by `\r` (carriage return) or `\n` (newline). The buffer is limited to 63 characters plus null terminator.

#### Functions and modularity

- `SerialInitialise(...)`: Configures the GPIO and USART for communication.
- `SerialOutputChar(...)`: Sends a character over USART.
- `SerialOutputString(...)`: Sends a string over USART.
- `SerialGetChar(...)`: Receives a single character (blocking).
- `SerialInputLine(...)`: Receives and stores characters in a buffer until Enter is pressed.
- `__io_putchar`, `__io_getchar`: Redirects `printf` and `getchar` to work with USART.

#### Testing

Testing is performed by connecting to the microcontroller’s serial port using a terminal emulator (e.g., CuteCom or PuTTY). Typed characters are echoed and displayed after pressing Enter. The prompt repeats for the next string.

### Part B
This part enhances the system with a receive callback mechanism, enabling the program to respond automatically once a full line is received, promoting an event-driven structure.

#### Usage

When the program runs, it waits for a user input line and then triggers a callback that displays two lines:
- One echoing what the user typed
- Another reprinting the same string for clarity

The user is then prompted again to enter the next input.

#### Valid input

Any string of characters terminated by `\r` (carriage return) or `\n` (newline). Input is stored in a buffer of size 64 (63 characters + null terminator).

#### Functions and modularity

- `All Part A functions.`
- `SerialSetReceiveCallback(...)`: Registers a function to be called when a full line is received.
- `OnLineReceived(char *string, uint32_t length)`: Callback function

#### Testing

Testing is performed similar to part A.

### Part C

This part implements interrupt-driven reception, allowing background input handling. When a line ends with `\n`, a callback is triggered to echo the input. This removes the need for polling.

#### Usage

Users type lines into the terminal. After Enter, input is echoed twice via the callback. Input handling occurs via interrupts.

#### Valid input

Any character sequence terminated by `\r` or `\n`. Buffer size is 64 characters.

#### Functions and modularity

- `Has all functions from previous parts.`
- `USART1_EXTI25_IRQHandler(...)`:  
  Interrupt handler that reads incoming characters and processes them, calling the callback when a complete line is received.

#### Testing

Testing is performed similar to parts A and B.

### Part D

This part enhances the serial communication system by enabling **non-blocking transmission**. Instead of printing strings character by character using blocking calls (like in `printf`), this version offloads the job to a transmit interrupt handler. The `SerialStartTransmission()` function initiates the process, and transmission continues in the background via interrupts. This makes the system more responsive and efficient, allowing background work while output is still occurring.

#### Usage

On start, a prompt is displayed using non-blocking transmission:
```
USART1 is ready. Type a line and press Enter:
```
User input is echoed back using the interrupt-based output handler.

#### Valid input

- Any string of characters terminated by a newline character (`\n`) will be accepted.
- Carriage return characters (`\r`) are ignored and do not terminate the input.
- The maximum accepted input length is 63 characters (due to the 64-byte input buffer including null terminator).
- Output strings are also limited by the 80-character `response` buffer in `OnLineReceived`.
- Input longer than the buffer length will be truncated.

#### Functions and modularity

- `All previous functions.`

- `SerialInitialise(...)`: Extended to support optional TX completion function.

- `SerialStartTransmission(const char *str)`  
  Starts a non-blocking transmission of a string using interrupts. The string is stored globally and transmitted one character at a time in the interrupt service routine.

- `USART1_EXTI25_IRQHandler(void)`  
  Combined interrupt handler:
  - Handles reception of characters: builds the input string, echoes characters, and triggers the receive callback when a newline is detected.
  - Handles transmission of characters: continues sending the current buffer until completion, then disables the transmit interrupt.

#### Testing
Connect using a terminal emulator. The program echoes lines typed by the user, and prompts are printed using background transmission.

## Timer Interface
The timer module enables the use of periodic and one-shot events using hardware timers. Timer 2 (TIM2) is primarily used in this module, however other timers can be enabled as well with minor changes. This code is designed to trigger user-defined callback functions after configurable delays while allowing other processes to be run simultaneously without having to use polling which takes away program time.
### Timer
The init_timer_module() function initialises a hardware timer with 1ms ticks by setting the prescaler value to 7999 which converts the timer clock to 8Mhz / (7999+1) = 1kHz and stores a callback function to a function pointer to be called whenever the timer reaches a desired count. This is achieved by configuring auto-reload register (ARR) to overflow after a user-defined value. It has two arguments, the specific timer to be intiialised (e.g. TIM2), and the callback function (e.g. blink_all_leds).
```
// Store a pointer to the function that is called when a timer interrupt occurs
void (*on_timer_interrupt)() = 0x00;

// Initialise timer with 1ms ticks to trigger a callback function regularly
// Input: desired timer number to initialise; callback function
void init_timer_module(TIM_TypeDef *TIM, void (*timer_callback)());
```
The current timer period can be accessed and changed by using the get_timer_period() and set_timer_period() functions which takes a specific timer number (e.g. TIM2) and either returns that timer's period in ms or set a user-defined period in ms for that timer. Timers' count can also be reset with a new period by the reset_timer() function which takes as input the timer number (e.g. TIM2) and the new period in ms (e.g. 1000).
```
// Function to reset a specific timer's count with a new period in ms
// Assuming timers are configured to 1kHz so each count is ms
void reset_timer(TIM_TypeDef *TIM, uint32_t new_period);
```
To execute the user-defined callback function stored in the function pointer on_timer_interrupt during initialisation, hardware interrupt is enabled by calling enable_timer2_interrupt() on initialisation:
```
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
```
It enables the update interrupt for TIM2 which is triggered whenever the timer overflows the ARR value (i.e. user-defined interval in ms) which then branches to execute the Interrupt Service Routine (ISR) for that timer. To enable hardware interrupt for other timers simply change "TIM2" in code above to the desired timer number. The function pointer on_timer_interrupt stores the address of the user callback, which is then called during the timer interrupt service routine (ISR):
```
// Interrupt Service Routine
void TIM2_IRQHandler(void) {
    // Check if the TIM2 interrupt flag is set
    if (TIM2->SR & TIM_SR_UIF) {
	// Run the callback function (make sure it is not null first)
        if (on_timer_interrupt != 0x00) {
		on_timer_interrupt();
	}
	...
        // Clear the interrupt flag (write 1 to the UIF bit to reset it)
        TIM2->SR &= ~TIM_SR_UIF;
    }
}
```
The handler first checks if the interrupt event is an update interrupt event as hardware interrupts can be triggered by any global timer event. If true, meaning the interrupt stems from the user-defined interval having elapsed, then the callback function is executed and the interrupt flag is cleared. By design the ISR will be triggered at regular intervals indefinitely.

### One Shot
The one_shot() function configures a specific timer in one-pulse mode (OPM) to execute a callback only once after a specified delay:
```
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
```
The OPM function uses the capture/compare (CC) event and CC interrupt to accomplish one-shot operation. The configuration follows instructions from the detailed manual for the STM32F3DISCOVERY board. The delay is determined by setting the CC register to the number of 1ms ticks and CC interrupt is enabled for hardware interrupt. This implementation prevents the program from occupying CPU time due to polling when compared to software solutions.

Then the current count is equal to the CC register value, the CC interrupt is raised and branches into the same ISR as in previous demonstration. The callback function is executed and the timer is stopped. The CC flag and interrupt is then disabled to prevent false triggers in future timer operation.
```
...
// If timer is in one-pulse mode reset timer to default mode
if (TIM2->SR & TIM_SR_CC1IF) {
	// Disable capture/compare flag
	TIM2->SR &= ~TIM_SR_CC1IF;
	// Disable the Capture/Compare 1 interrupt
	TIM2->DIER &= ~TIM_DIER_CC1IE;
}
...
```
### Testing
The two main functions of the timer module (i.e. configurable delay and user-defined callback function) can be tested through changing the delay variable in main.c or in functions init_timer_module() where onboard LEDs will provide a visual aid for proper operations. 
It is assumed all inputs will be logical and any callback function properly defined. 
Delay time should not exceed 2^32-1 or be negative.
#### Callback function triggered at reguar intervals
Input:

```
...
uint32_t delay = 1000;
init_timer_module(TIM2, blink_all_leds);
...
```

Output:
All LEDs will be on 1000ms/1s and off 1000ms/1s indefinitely.

Input:

```
...
uint32_t delay = 50;
init_timer_module(TIM2, blink_alternate_leds);
...
```

Output:
Every second LED will be on 50ms/0.05s and off 50ms/0.05s indefinitely.

#### One-shot mode
Input:
```
...
uint32_t delay = 1000;
init_timer_module(TIM2, blink_all_leds);
...
```
Output:
All LEDs will be turned on and after a 1000ms/1s delay all will turn off and stay off indefintely.

Input:

```
...
uint32_t delay = 50;
init_timer_module(TIM2, blink_alternate_leds);
...
```

Output:
Every second LED will be turned on and after a 50ms/0.05s delay all will turn off and stay off indefintely.

## Integration 
The integration tasks encapsulated all 3 modules together to provide a working user interface that can control each function and certain parameters through input into the laptop. All modules have stayed the same as what was explained above, however, there have been some slight changes to ensure proper functionality of the task.
![Image](https://github.com/user-attachments/assets/32abd551-3bde-4d2e-8db0-c6ee30dced27)

This flowchart depicts the integration task and key compoennts in its functionality.  

### Main Function

The main function handles the user interface by interpreting and responding to input commands received over the serial connection. It serves as the integration point for all three modules (Digital IO, Timer, and Serial Output), coordinating their responses based on the user input.

- **Input Validation via If-Else Logic**  
  When a line of input is received, the `OnLineReceived()` function is triggered via an interrupt-based callback. It uses an `if-else` structure to determine whether the entered command is valid. If the input cannot be split into a command and argument using `sscanf`, it responds with “Invalid input format.” Valid commands like `led`, `serial`, `timer`, or `oneshot` are then handled accordingly, while anything else results in an “Unknown command.”

- **Command Parsing and Buffer Handling**  
  The function uses two static character buffers (`command` and `argument`) to store the parsed parts of the input. The `sscanf()` function separates the command (e.g., `led`) and the argument (e.g., `10101010`) from the full input string. These buffers act like a form of double-buffering—separating the user input into reusable storage for each part of the instruction—allowing the command logic to remain clean and modular. The `argument` is then passed into the relevant module's function (e.g., `DigitalIO_SetPattern()`).

- **Struct-Based Mode Management**  
  The `SystemMode` enum and `current_mode` variable provide a simple state machine for the program. Each time a new valid command is processed, `reset_all_modes()` is called first to disable any previously active mode, ensuring that only one module operates at a time. This prevents conflicts like overlapping timer outputs or LED states. Then, `current_mode` is updated based on the new user command, which can be used for any required state-dependent behavior or diagnostics.

This design allows each module to operate independently but still work together under a unified command structure, making the system responsive, robust, and easy to expand.



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
Testing of the Integration method was done by comparing the observed results to the expected results. If there was a difference between the two, this meant that functiality was not efficient and the code was debugged to solve the issue: 

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
    <td>All LEDs should be off and then blink with an interval of 1 second</td>
  </tr>
  <tr>
    <td>timer 3000</td>
    <td>All LEDs should be off and then blink with an interval of 3 seconds</td>
  </tr>

  <!-- One Shot -->
  <tr>
    <td rowspan="2"><b>One Shot</b></td>
    <td>oneshot 1000</td>
    <td>All LEDs should be off and then turn and stay on after a delay of 1 second</td>
  </tr>
  <tr>
    <td>oneshot 3000</td>
    <td>All LEDs should be off and then turn on and stay on after a delay of 3 second</td>
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


