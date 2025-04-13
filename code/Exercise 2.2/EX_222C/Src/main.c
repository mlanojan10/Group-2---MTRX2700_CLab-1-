#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "stm32f303xc.h"

void OnLineReceived(char *string, uint32_t length) {
    printf("> You typed: %s\r\n", string);
    printf("> Echo: %s\r\n\r\n", string);
    printf("> ");  // Print prompt again
}

int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT); // Send character to USART1
    return ch;
}

int main(void) {
	// Initialize USART1 at 115200 baud rate with default settings
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);

    // Register the callback function for received lines
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);

    __enable_irq();  // Enable global interrupts

    // Initial welcome message
    printf("USART1 is ready. Type a line and press Enter:\r\n");
    printf("> ");

    while (1) {
        // Nothing needed here – receive is handled via interrupts
    }
}
