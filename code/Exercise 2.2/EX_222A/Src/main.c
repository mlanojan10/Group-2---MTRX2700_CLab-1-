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

int main(void) {
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    printf("USART1 is ready. Type a line and press Enter:\r\n");

    char input_buffer[64];

    while (1) {
		printf("> ");  // Print prompt
		SerialInputLine(input_buffer, sizeof(input_buffer), &USART1_PORT);  // Read input
		printf("Echo: %s\r\n\r\n", input_buffer);  // Echo back with extra newline
    }

}
