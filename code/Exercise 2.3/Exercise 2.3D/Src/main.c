#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "stm32f303xc.h"

int __io_putchar(int ch) {
    SerialOutputChar((uint8_t)ch, &USART1_PORT);
    return ch;
}

void OnLineReceived(char *string, uint32_t length) {
    static char response[80];
    snprintf(response, sizeof(response), "> You typed: %s\r\n> String: %s\r\n\r\n", string, string);
    SerialStartTransmission(response);
}

int main(void) {
    SerialInitialise(BAUD_115200, &USART1_PORT, NULL);
    SerialSetReceiveCallback(&USART1_PORT, OnLineReceived);

    printf("USART1 is ready. Type a line and press Enter:\r\n> ");

    while (1) {
        // UART handled via interrupts
    }
}
