#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"
#include "timer.h"
#include "digitalio.h"
#include "stm32f303xc.h"

#define MAX_CMD_LEN 64

int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

void OnLineReceived(char *input, uint32_t len) {
    char command[MAX_CMD_LEN], argument[MAX_CMD_LEN];
    if (sscanf(input, "%s %[^\n]", command, argument) != 2) {
        SerialStartTransmission("Invalid input format.\r\n> ");
        return;
    }

    if (strcmp(command, "led") == 0) {
        DigitalIO_SetPattern(argument);
        SerialStartTransmission("LED pattern set.\r\n> ");
    }
    else if (strcmp(command, "serial") == 0) {
        SerialStartTransmission(argument);
        SerialStartTransmission("\r\n> ");
    }
    else if (strcmp(command, "timer") == 0) {
        uint32_t ms = atoi(argument);
        Timer_SetPeriodic(ms);
        SerialStartTransmission("Periodic timer started.\r\n> ");
    }
    else if (strcmp(command, "oneshot") == 0) {
        uint32_t ms = atoi(argument);
        Timer_TriggerOneShot(ms);
        SerialStartTransmission("One-shot timer triggered.\r\n> ");
    }
    else {
        SerialStartTransmission("Unknown command.\r\n> ");
    }
}

int main(void) {
    enable_clocks();                       // Enable GPIO/TIM clocks
    initialise_board();                    // Configure LEDs
    timer_enable_clocks();
    timer_initialise_board();

    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);
    enable_timer2_interrupt();             // Enable TIM2 IRQs
    pattern_callback_init(display_pattern_callback); // Register LED pattern handler

    printf("USART1 is ready. Type a line and press Enter:\r\n> ");

    while (1) {
        // Idle - everything runs via interrupts
    }
}
