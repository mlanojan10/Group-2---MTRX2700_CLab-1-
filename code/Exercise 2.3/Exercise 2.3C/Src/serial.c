#include "serial.h"
#include <string.h>
#include "stm32f303xc.h"

// Structure to hold serial port configuration and callback functions
struct _SerialPort {
    USART_TypeDef *UART;           // Pointer to USART peripheral (e.g., USART1)
    GPIO_TypeDef *GPIO;            // Pointer to GPIO port for TX/RX pins
    volatile uint32_t MaskAPB2ENR;        // Bitmask for enabling USART on APB2 bus
    volatile uint32_t MaskAPB1ENR;        // Bitmask for enabling USART on APB1 bus
    volatile uint32_t MaskAHBENR;         // Bitmask for enabling GPIO on AHB bus
    volatile uint32_t SerialPinModeValue;                // Pin mode configuration
    volatile uint32_t SerialPinSpeedValue;               // Pin speed configuration
    volatile uint32_t SerialPinAlternatePinValueLow;     // AF setting for AFR[0] (pins 0–7)
    volatile uint32_t SerialPinAlternatePinValueHigh;    // AF setting for AFR[1] (pins 8–15)
    void (*completion_function)(uint32_t);               // Optional: Called when string is sent
    void (*receive_callback)(char *, uint32_t);          // Called when a line is received
};

// Global buffer and index for RX
static char rx_buffer[64];           // Stores received characters until newline
static uint32_t rx_index = 0;        // Current index into rx_buffer


// Definition of USART1 port configuration
SerialPort USART1_PORT = {
    USART1,                      // USART1 peripheral
    GPIOC,                       // GPIO port C
    RCC_APB2ENR_USART1EN,        // USART1 clock on APB2
    0x00,                        // No APB1 peripheral to enable
    RCC_AHBENR_GPIOCEN,          // Enable GPIOC clock
    0xA00,                       // MODER settings for alternate function
    0xF00,                       // OSPEEDR settings for speed
    0x770000,                    // AFR[0] alternate function setting for pins 4 and 5
    0x00,                        // AFR[1] not used
    0x00                          // No completion function yet
};

// Initializes the specified serial port for transmission and reception
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {
    serial_port->completion_function = completion_function;

    // Enable clocks for power interface and system config controller
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHBENR |= serial_port->MaskAHBENR;	// Enable clock for the GPIO port


    // Configure GPIO pins for alternate function, speed, and AF value
    serial_port->GPIO->MODER = serial_port->SerialPinModeValue;
    serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;
    serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;
    serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;


    // Enable USART peripheral clocks
    RCC->APB1ENR |= serial_port->MaskAPB1ENR;
    RCC->APB2ENR |= serial_port->MaskAPB2ENR;

    // Set baud rate manually to 115200 at 8MHz (BRR = 0x46)
    uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR;
    *baud_rate_config = 0x46;  // 115200 at 8MHz

    // Enable transmitter, receiver, and USART
    serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // Enable RX interrupt
    serial_port->UART->CR1 |= USART_CR1_RXNEIE;

    // Enable USART1 interrupt in the NVIC
    NVIC_EnableIRQ(USART1_IRQn);
}


// Transmits a single character through the serial port
void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
	// Wait until TX register is empty
    while((serial_port->UART->ISR & USART_ISR_TXE) == 0);
    serial_port->UART->TDR = data; // Transmit character
}

// Sends a null-terminated string via the serial port
void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {
    uint32_t counter = 0;
    while(*pt) {
        SerialOutputChar(*pt, serial_port); // Send character
        counter++;
        pt++;
    }
    // Call completion callback, if one is set
    if (serial_port->completion_function)
        serial_port->completion_function(counter);
}

// Blocking function that waits for and returns a received character
uint8_t SerialGetChar(SerialPort *serial_port) {
    while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0);
    return serial_port->UART->RDR; // Return received character
}

// Registers a callback function to be called when a line is received
void SerialSetReceiveCallback(SerialPort *serial_port, void (*callback)(char *, uint32_t)) {
    serial_port->receive_callback = callback;
}

// Interrupt handler for USART1 (triggered on character reception or errors)
void USART1_EXTI25_IRQHandler(void)
{

	// If RXNE flag is set, a character has been received
    if (USART1->ISR & USART_ISR_RXNE) {
        char c = USART1->RDR;	// Read the received character

        if (c == '\r') return;  // Ignore carriage return (CR)

        SerialOutputChar(c, &USART1_PORT);  // Echo character back to sender

        if (c == '\n') { // End of line received
            rx_buffer[rx_index] = '\0'; // Null-terminate the buffer
            if (USART1_PORT.receive_callback)
                USART1_PORT.receive_callback(rx_buffer, rx_index); // Call line handler
            rx_index = 0;  // Reset buffer index for next line
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = c; // Store character in buffer
        }
    }

    // If overrun error (ORE) occurred, clear it by reading RDR
    if (USART1->ISR & USART_ISR_ORE) {
        (void)USART1->RDR; // Clear Overrun by reading
    }
}

