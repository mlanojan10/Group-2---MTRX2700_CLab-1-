#include "serial.h"
#include <string.h>
#include "stm32f303xc.h"

struct _SerialPort {
    USART_TypeDef *UART;
    GPIO_TypeDef *GPIO;
    volatile uint32_t MaskAPB2ENR;
    volatile uint32_t MaskAPB1ENR;
    volatile uint32_t MaskAHBENR;
    volatile uint32_t SerialPinModeValue;
    volatile uint32_t SerialPinSpeedValue;
    volatile uint32_t SerialPinAlternatePinValueLow;
    volatile uint32_t SerialPinAlternatePinValueHigh;
    void (*completion_function)(uint32_t);
    void (*receive_callback)(char *, uint32_t);
};

// Global buffer and index for RX
static char rx_buffer[64];
static uint32_t rx_index = 0;

SerialPort USART1_PORT = {
    USART1,
    GPIOC,
    RCC_APB2ENR_USART1EN,
    0x00,
    RCC_AHBENR_GPIOCEN,
    0xA00,
    0xF00,
    0x770000,
    0x00,
    0x00
};

void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {
    serial_port->completion_function = completion_function;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHBENR |= serial_port->MaskAHBENR;

    serial_port->GPIO->MODER = serial_port->SerialPinModeValue;
    serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;
    serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;
    serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;

    RCC->APB1ENR |= serial_port->MaskAPB1ENR;
    RCC->APB2ENR |= serial_port->MaskAPB2ENR;

    uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR;
    *baud_rate_config = 0x46;  // 115200 at 8MHz

    serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // Enable RX interrupt
    serial_port->UART->CR1 |= USART_CR1_RXNEIE;

    // Enable IRQ in NVIC
    NVIC_EnableIRQ(USART1_IRQn);
}

void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
    while((serial_port->UART->ISR & USART_ISR_TXE) == 0);
    serial_port->UART->TDR = data;
}

void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {
    uint32_t counter = 0;
    while(*pt) {
        SerialOutputChar(*pt, serial_port);
        counter++;
        pt++;
    }
    if (serial_port->completion_function)
        serial_port->completion_function(counter);
}

uint8_t SerialGetChar(SerialPort *serial_port) {
    while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0);
    return serial_port->UART->RDR;
}

void SerialSetReceiveCallback(SerialPort *serial_port, void (*callback)(char *, uint32_t)) {
    serial_port->receive_callback = callback;
}

void USART1_EXTI25_IRQHandler(void)
{

    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;

        if (c == '\r') return;  // Ignore CR

        SerialOutputChar(c, &USART1_PORT);  // Echo back

        if (c == '\n') {
            rx_buffer[rx_index] = '\0';
            if (USART1_PORT.receive_callback)
                USART1_PORT.receive_callback(rx_buffer, rx_index);
            rx_index = 0;
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = c;
        }
    }

    if (USART1->ISR & USART_ISR_ORE) {
        (void)USART1->RDR; // Clear Overrun
    }
}

