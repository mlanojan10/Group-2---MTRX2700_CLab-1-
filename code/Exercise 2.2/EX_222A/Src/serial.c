#include "serial.h"
#include <string.h>

#include "stm32f303xc.h"

// Structure defining parameters for a specific serial port
// We store the pointers to the GPIO and USART that are used
//  for a specific serial port. To add another serial port
//  you need to select the appropriate values.
struct _SerialPort {
	USART_TypeDef *UART;  // Pointer to the USART hardware
	GPIO_TypeDef *GPIO;   // Pointer to the GPIO port used
	volatile uint32_t MaskAPB2ENR;	// mask to enable RCC APB2 bus registers
	volatile uint32_t MaskAPB1ENR;	// mask to enable RCC APB1 bus registers
	volatile uint32_t MaskAHBENR;	// mask to enable RCC AHB bus registers
	volatile uint32_t SerialPinModeValue;        // Value to set pin mode to Alternate Function
	volatile uint32_t SerialPinSpeedValue;       // Value to set high speed for GPIO pins
	volatile uint32_t SerialPinAlternatePinValueLow;   // Low register for alternate function
	volatile uint32_t SerialPinAlternatePinValueHigh;  // High register for alternate function
	void (*completion_function)(uint32_t);             // Optional callback after transmission
};


// Define an instance of SerialPort for USART1 (PC10 and PC11 used)
// instantiate the serial port parameters
//   note: the complexity is hidden in the c file
SerialPort USART1_PORT = {
		USART1,               // USART1 hardware
		GPIOC,                // GPIOC used for TX/RX
		RCC_APB2ENR_USART1EN, // Enable USART1 on APB2 bus
		0x00,	              // No APB1 usage
		RCC_AHBENR_GPIOCEN,   // Enable GPIOC on AHB bus
		0xA00,                // Set PC10, PC11 to Alternate Function mode
		0xF00,                // Set PC10, PC11 to high speed
		0x770000,             // Alternate function mapping for PC10 and PC11 (AF7)
		0x00,                 // No change to high AFR
		0x00                  // No completion function by default
		};


// Initialises the specified serial port with a baud rate and optional completion function
// InitialiseSerial - Initialise the serial port
// Input: baudRate is from an enumerated set
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {

	serial_port->completion_function = completion_function; // Set the callback

	// Enable power interface and system configuration controller
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
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_19200:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_38400:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_57600:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_115200:
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	}


	// enable serial port for tx and rx
	serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}


// Sends a single byte over the USART
void SerialOutputChar(uint8_t data, SerialPort *serial_port) {

	while((serial_port->UART->ISR & USART_ISR_TXE) == 0){ // Wait until transmit buffer is empty
	}

	serial_port->UART->TDR = data; // Write data to transmit data register
}


// Sends a null-terminated string over the USART
void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {

	uint32_t counter = 0;
	while(*pt) {
		SerialOutputChar(*pt, serial_port); // Send each character
		counter++;
		pt++;
	}

	// Call the optional completion function with number of bytes sent
	serial_port->completion_function(counter);
}


// Receives a single byte from the USART (blocking)
uint8_t SerialGetChar(SerialPort *serial_port) {
	while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0); // Wait until data is received
	return serial_port->UART->RDR;                          // Return received byte
}


// Receives a full line of input from the serial port, terminated by newline or carriage return
void SerialInputLine(char *buffer, uint32_t max_len, SerialPort *serial_port) {
    uint32_t i = 0;

    while (i < max_len - 1) {
        char c = SerialGetChar(serial_port);   // Receive character

        // Echo character back
        SerialOutputChar(c, serial_port);

        if (c == '\r' || c == '\n') {   // Stop if Enter is pressed
            break;
        }

        buffer[i++] = c;   // Store character in buffer
    }

    buffer[i] = '\0'; // Null-terminate the string
}
