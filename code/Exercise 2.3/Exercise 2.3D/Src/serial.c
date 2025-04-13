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

static char rx_buffers[2][64];
static uint8_t active_rx_buf = 0;
static uint32_t rx_index = 0;

static const char *tx_buffer = NULL;
static uint32_t tx_index = 0;

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
    *baud_rate_config = 0x46;

    serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    serial_port->UART->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART1_IRQn);
}

void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
    while ((serial_port->UART->ISR & USART_ISR_TXE) == 0);
    serial_port->UART->TDR = data;
}

void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {
    while (*pt) {
        SerialOutputChar(*pt++, serial_port);
    }
}

uint8_t SerialGetChar(SerialPort *serial_port) {
    while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0);
    return serial_port->UART->RDR;
}

void SerialSetReceiveCallback(SerialPort *serial_port, void (*callback)(char *, uint32_t)) {
    serial_port->receive_callback = callback;
}

void SerialStartTransmission(const char *str) {
    tx_buffer = str;
    tx_index = 0;
    USART1->CR1 |= USART_CR1_TXEIE;
}

void USART1_EXTI25_IRQHandler(void) {
    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;
        SerialOutputChar(c, &USART1_PORT);

        if (c == '\r') return;

        if (c == '\n') {
            rx_buffers[active_rx_buf][rx_index] = '\0';
            if (USART1_PORT.receive_callback)
                USART1_PORT.receive_callback(rx_buffers[active_rx_buf], rx_index);
            active_rx_buf ^= 1;
            rx_index = 0;
        } else if (rx_index < sizeof(rx_buffers[0]) - 1) {
            rx_buffers[active_rx_buf][rx_index++] = c;
        }
    }

    if ((USART1->CR1 & USART_CR1_TXEIE) && (USART1->ISR & USART_ISR_TXE)) {
        if (tx_buffer && tx_buffer[tx_index]) {
            USART1->TDR = tx_buffer[tx_index++];
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;
            tx_buffer = NULL;
            tx_index = 0;
        }
    }
}
