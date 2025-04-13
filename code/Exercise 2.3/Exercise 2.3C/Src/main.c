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
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

int main(void) {
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);

    __enable_irq();  // Ensure global interrupts are on

    printf("USART1 is ready. Type a line and press Enter:\r\n");
    printf("> ");

    while (1) {
        // Nothing needed here – receive is handled via interrupts
    }
}
