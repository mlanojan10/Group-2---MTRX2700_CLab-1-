#include "serial.h"
#include <string.h>
#include "stm32f303xc.h"

// Structure defining configuration for a serial port
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
    USART1, GPIOC,
    RCC_APB2ENR_USART1EN, 0x00, RCC_AHBENR_GPIOCEN,
    0xA00, 0xF00,
    0x770000, 0x00,
    0x00, 0x00
};

// Double buffer for received input lines
static char rx_buffers[2][64];
static uint8_t active_rx_buf = 0;   // Active buffer index
static uint32_t rx_index = 0;       // Index into current buffer

// Pointer to currently transmitting string
static const char *tx_buffer = NULL;
static uint32_t tx_index = 0;

// Initializes UART hardware and GPIO settings for communication
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {
    serial_port->completion_function = completion_function;

    // Enable required clocks
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHBENR |= serial_port->MaskAHBENR;

    // Configure GPIO alternate function mode and speed
    serial_port->GPIO->MODER = serial_port->SerialPinModeValue;
    serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;
    serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;
    serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;

    // Enable USART peripheral
    RCC->APB1ENR |= serial_port->MaskAPB1ENR;
    RCC->APB2ENR |= serial_port->MaskAPB2ENR;

    // Set baud rate (hardcoded value for 115200 @ 8 MHz)
    uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR;
    *baud_rate_config = 0x46;

    // Enable TX, RX, UART and RX interrupt
    serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    serial_port->UART->CR1 |= USART_CR1_RXNEIE;

    // Enable USART1 interrupt in NVIC
    NVIC_EnableIRQ(USART1_IRQn);
}

// Transmit a single character (blocking)
void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
    while ((serial_port->UART->ISR & USART_ISR_TXE) == 0);  // Wait for TX to be ready
    serial_port->UART->TDR = data;                          // Write to transmit register
}

// Transmit a null-terminated string using SerialOutputChar
void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {
    while (*pt) {
        SerialOutputChar(*pt++, serial_port);
    }
}

// Blocking read of a single character from UART
uint8_t SerialGetChar(SerialPort *serial_port) {
    while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0);  // Wait for data
    return serial_port->UART->RDR;                           // Return received char
}

// Sets the function to be called when a full line is received via UART
void SerialSetReceiveCallback(SerialPort *serial_port, void (*callback)(char *, uint32_t)) {
    serial_port->receive_callback = callback;
}

// Begins transmission of a string (interrupt-based, non-blocking)
void SerialStartTransmission(const char *str) {
    tx_buffer = str;
    tx_index = 0;
    USART1->CR1 |= USART_CR1_TXEIE;  // Enable TX interrupt
}

// UART interrupt handler (for RX and TX)
void USART1_EXTI25_IRQHandler(void) {
    // ---------- Receive ----------
    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;  // Read received character
        SerialOutputChar(c, &USART1_PORT);  // Echo it back

        // If Enter pressed, complete the string
        if (c == '\r') return;

        if (c == '\n') {
            rx_buffers[active_rx_buf][rx_index] = '\0';  // Null terminate

            if (USART1_PORT.receive_callback)
                USART1_PORT.receive_callback(rx_buffers[active_rx_buf], rx_index);

            active_rx_buf ^= 1;  // Switch buffer
            rx_index = 0;        // Reset index
        } else if (rx_index < sizeof(rx_buffers[0]) - 1) {
            rx_buffers[active_rx_buf][rx_index++] = c;
        }
    }

    // ---------- Transmit ----------
    if ((USART1->CR1 & USART_CR1_TXEIE) && (USART1->ISR & USART_ISR_TXE)) {
        if (tx_buffer && tx_buffer[tx_index]) {
            USART1->TDR = tx_buffer[tx_index++];  // Send next character
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;  // Disable TX interrupt
            tx_buffer = NULL;
            tx_index = 0;
        }
    }
}
