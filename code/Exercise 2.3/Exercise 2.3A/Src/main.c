#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "stm32f303xc.h"

// Redirects the standard output (e.g., printf) to USART1
int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT); // Send character via USART1
    return ch;
}

// Redirects standard input (e.g., getchar) to USART1
int __io_getchar(void) {
    return SerialGetChar(&USART1_PORT); // Receive a character from USART1
}

int main(void) {
	// Initialize USART1 with a baud rate of 115200. The third parameter is
	// NULL (no callback function).
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    // Inform the user that USART1 is ready and prompt for input
    printf("USART1 is ready. Type a line and press Enter:\r\n");

    // Declare a buffer to hold user input (max 63 characters + null terminator)
    char input_buffer[64];

    // Infinite loop to continuously read and echo input
    while (1) {
		printf("> ");  // Print prompt
		SerialInputLine(input_buffer, sizeof(input_buffer), &USART1_PORT);  // Read an entire line of input from USART1
		printf("String: %s\r\n\r\n", input_buffer);  // Echo back with extra newline
    }

}
