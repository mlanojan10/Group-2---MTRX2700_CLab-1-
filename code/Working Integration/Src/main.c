#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"
#include "timer.h"
#include "digitalio.h"
#include "stm32f303xc.h"

#define MAX_CMD_LEN 64               // Maximum length of command and argument strings

int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

// Declare mode enum - this sets the mode of each module so when a new user input is added
// It overrides the existing mode for correct functionality
typedef enum {
    MODE_NONE,
    MODE_LED,
    MODE_TIMER,
    MODE_ONESHOT,
    MODE_SERIAL
} SystemMode;

SystemMode current_mode = MODE_NONE;   // Global variable to track current system mode

// Function to resets everything back to default state, so a new mode can be set
void reset_all_modes() {
    TIM2->CR1 = 0;              // Stop Timer 2
    leds_set_state(0);          // Turn off all LEDs
    USART1->CR1 &= ~USART_CR1_TXEIE; // Disable USART1 TX interrupt
    current_mode = MODE_NONE;   // Reset current mode state
}

void OnLineReceived(char *input, uint32_t len) {
    static char command[MAX_CMD_LEN], argument[MAX_CMD_LEN];

    // Parse input into a command and an argument using sscanf
    // This splits the command and instructions
    if (sscanf(input, "%s %[^\n]", command, argument) != 2) {
        SerialStartTransmission("Invalid input format.\r\n> ");
        return;
    }
    reset_all_modes();     // Reset all modes so a new one can be chosen

    // The command is then checked against set names to switch between modules
    // The instructions are passed as an argument to the chosen module to complete the task

    //DIGITALIO Module
    if (strcmp(command, "led") == 0) {      // If command is "led" complete DIGITALIO Module
        //Integration Main Functionality
    	DigitalIO_SetPattern(argument);

    	//Testing Function - without timer:
    	//button_init(chase_led);
    	//button_init(test_callback);

    	//To show function with timer embedded, please refer to code within EX_222D
    	//There is issues with having TIM2_IRQHandler defined twice in the project for the Timer Interface and this

        current_mode = MODE_LED;            // Update system mode
        SerialStartTransmission("LED pattern set.\r\n> ");
    }

    // SERIAL Module
    else if (strcmp(command, "serial") == 0) {     // If command is "serial" complete SERIAL Module
        current_mode = MODE_SERIAL;               // Update system mode

        static char formatted[80];  // Static buffer to hold formatted string
        snprintf(formatted, sizeof(formatted), "String: %s \n>", argument); //Show string to User

        SerialStartTransmission(formatted);
    }

    // TIMER Module (timer)
    else if (strcmp(command, "timer") == 0) {   // If command is "timer" complete TIMER Module
        uint32_t ms = atoi(argument);           // Convert argument to integer milliseconds
        current_mode = MODE_TIMER;
        reset_timer(ms);                  // Start periodic timer with that interval
        SerialStartTransmission("Periodic timer started.\r\n> ");
    }

    // TIMER Module (Oneshot)
    else if (strcmp(command, "oneshot") == 0) {
        uint32_t ms = atoi(argument);         // Convert argument to integer milliseconds
        one_shot(ms);             // Trigger one-shot timer event
        current_mode = MODE_ONESHOT;
        SerialStartTransmission("One-shot timer triggered.\r\n> ");
    }

    // If command is unknown, notify user and do nothing
    else {
        SerialStartTransmission("Unknown command.\r\n> ");
    }
}

int main(void) {
    enable_clocks();                       // Enable GPIO/TIM clocks
    initialise_board();                    // Configure LEDs
    timer_enable_clocks();                 // Enable Timer-related clocks
    timer_initialise_board();              // Initialise Timer peripheral

    // Initialise Serial communication over USART1
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);


    enable_timer2_interrupt();             // Enable TIM2 IRQs
    pattern_callback_init(display_pattern_callback); // Register LED pattern handler

    printf("USART1 is ready. Type a line and press Enter:\r\n> "); //Notify user that system is ready

    while (1) {
        // Idle - everything runs via interrupts
    }
}
