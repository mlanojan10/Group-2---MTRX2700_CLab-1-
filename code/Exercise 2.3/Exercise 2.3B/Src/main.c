#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "stm32f303xc.h"

int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

int __io_getchar(void) {
    return SerialGetChar(&USART1_PORT);
}

void OnLineReceived(char *string, uint32_t length) {
    printf("> You typed: %s\r\n", string);
    printf("> String: %s\r\n\r\n", string);
}

int main(void) {
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived); // Register RX callback
    printf("USART1 is ready. Type a line and press Enter:\r\n");

    char input_buffer[64];

    while (1) {
		printf("> ");  // Print prompt
		SerialInputLine(input_buffer, sizeof(input_buffer), &USART1_PORT);  // Read input
    }

}
