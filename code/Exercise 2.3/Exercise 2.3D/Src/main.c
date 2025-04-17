#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "stm32f303xc.h"

int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

// Callback function called when a full line is received via UART
void OnLineReceived(char *string, uint32_t length) {
    static char response[80];

    // Format and store a response string including the user input
    snprintf(response, sizeof(response), 
             "> You typed: %s\r\n> String: %s\r\n\r\n", string, string);

    // Transmit the formatted response back over UART
    SerialStartTransmission(response);
}

int main(void) {
    // Initialise USART1 with 115200 baud rate
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);

    // Register the line-received callback
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);

    // Initial startup message
    printf("USART1 is ready. Type a line and press Enter:\r\n> ");

    while (1) {
        // Main loop remains idle; UART input/output handled by interrupts
    }
}
