#ifndef SERIAL_PORT_HEADER
#define SERIAL_PORT_HEADER

#include <stdint.h>

// Forward-declare the struct (definition is hidden in serial.c)
struct _SerialPort;
typedef struct _SerialPort SerialPort;

// Public USART1 handle
extern SerialPort USART1_PORT;

// Baud rate options
enum {
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200
};

// Initialise serial port with optional TX completion callback
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t));

// Output a single character (blocking)
void SerialOutputChar(uint8_t data, SerialPort *serial_port);

// Output a NULL-terminated string (blocking)
void SerialOutputString(uint8_t *pt, SerialPort *serial_port);

// Get a single character (blocking, not used in Q3 anymore)
uint8_t SerialGetChar(SerialPort *serial_port);

// OPTIONAL: legacy polling-based input (not used in Q3, can remove if unused)
void SerialInputLine(char *buffer, uint32_t max_len, SerialPort *serial_port);

// Register a callback to be triggered on full line received via interrupt
void SerialSetReceiveCallback(SerialPort *serial_port, void (*callback)(char *, uint32_t));

#endif
