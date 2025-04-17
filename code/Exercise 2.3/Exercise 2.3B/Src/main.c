#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "stm32f303xc.h"

// Enables printf for USART1
int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

// Enables getchar for USART1
int __io_getchar(void) {
    return SerialGetChar(&USART1_PORT);
}

// Callback function that is called when a full line is received over serial
void OnLineReceived(char *string, uint32_t length) {
    // Echo back the received string in formatted style
    printf("> You typed: %s\r\n", string);
    printf("> String: %s\r\n\r\n", string);
}

int main(void) {
    // Initialize USART1 with baud rate 115200
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);

    // Register a function to be called whenever a full line is received
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);

    // Print a ready message for the user
    printf("USART1 is ready. Type a line and press Enter:\r\n");

    // Buffer to store the received input string
    char input_buffer[64];

    while (1) {
        // Print a command prompt
        printf("> ");

        // Blocking read until a full line is received (terminated by '\n')
        SerialInputLine(input_buffer, sizeof(input_buffer), &USART1_PORT);
    }
}
