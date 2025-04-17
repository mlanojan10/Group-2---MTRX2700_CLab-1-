#include "serial.h"
#include <string.h>

#include "stm32f303xc.h"

//  Struct nto enable easy access to all the register addresses (enable masks etc)
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
	void (*receive_callback)(char *, uint32_t); // RX callback
};


//   Add serial port parameters (some definition values are in includes files)
SerialPort USART1_PORT = {USART1,
		GPIOC,
		RCC_APB2ENR_USART1EN, // bit to enable for APB2 bus
		0x00,	// bit to enable for APB1 bus
		RCC_AHBENR_GPIOCEN, // bit to enable for AHB bus
		0xA00,
		0xF00,
		0x770000,  // for USART1 PC10 and 11, this is in the AFR low register
		0x00, // no change to the high alternate function register
		0x00 // default function pointer is NULL
		};



// Initialise serial and set baud rate to 115200
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {

	serial_port->completion_function = completion_function;

	// enable clock power, system configuration clock and GPIOC
	// common to all UARTs
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// enable the GPIO which is on the AHB bus
	RCC->AHBENR |= serial_port->MaskAHBENR;

	// set pin mode to alternate function for the specific GPIO pins
	serial_port->GPIO->MODER = serial_port->SerialPinModeValue;

	// enable high speed clock for specific GPIO pins
	serial_port->GPIO->OSPEEDR = serial_port->SerialPinSpeedValue;

	// set alternate function to enable USART to external pins
	serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;
	serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;

	// enable the device based on the bits defined in the serial port definition
	RCC->APB1ENR |= serial_port->MaskAPB1ENR;
	RCC->APB2ENR |= serial_port->MaskAPB2ENR;

	// Get a pointer to the 16 bits of the BRR register that we want to change
	uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR; // only 16 bits used!

	// Baud rate calculation from datasheet
	switch(baudRate){
	case BAUD_9600:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_19200:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_38400:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_57600:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_115200:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	}


	// enable serial port for tx and rx
	serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}


// Transmits a single character over the specified UART port
void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
    // Wait for TDR to be ready
	while ((serial_port->UART->ISR & USART_ISR_TXE) == 0) {
	}

    // Write the character to the TDR register for transmission
	serial_port->UART->TDR = data;
}

// Transmits a null-terminated string over the specified UART port
void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {
	uint32_t counter = 0;

    // Send each character one at a time until the null terminator is found
	while (*pt) {
		SerialOutputChar(*pt, serial_port);  // Transmit character
		counter++;                           // Count characters sent
		pt++;                                // Advance pointer
	}

    // Call the optional completion callback (if set) to notify string length sent
	serial_port->completion_function(counter);
}

// Blocking receive of a single character over the specified UART port
uint8_t SerialGetChar(SerialPort *serial_port) {
    // Wait until a character is received (RXNE = 1)
	while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0);

    // Return the received character
	return serial_port->UART->RDR;
}

// Receives a full line of input from the UART until newline or carriage return
void SerialInputLine(char *buffer, uint32_t max_len, SerialPort *serial_port) {
    uint32_t i = 0;

    // Read characters one at a time
    while (i < max_len - 1) {
        char c = SerialGetChar(serial_port);      // Blocking read
        SerialOutputChar(c, serial_port);         // Echo character back to user

        if (c == '\r' || c == '\n') {
            break;                                // Stop reading on newline or carriage return
        }

        buffer[i++] = c;                          // Store character in buffer
    }

    buffer[i] = '\0';  // Null-terminate the string

    // If a receive callback has been registered, call it with the full line
    if (serial_port->receive_callback != NULL) {
        serial_port->receive_callback(buffer, i);
    }
}

// Sets the callback function to be called when a full line is received
void SerialSetReceiveCallback(SerialPort *serial_port, void (*callback)(char *, uint32_t)) {
    serial_port->receive_callback = callback;
}

