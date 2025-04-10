.syntax unified
.thumb

.global main

#include "initialise.s"

.data
@ define variables


.align

incoming_buffer: .space 62

user_defined_terminating_character: .asciz "#"


.text
@ define text


@ this is the entry function called from the c file
main:

	BL initialise_power
	BL enable_peripheral_clocks
	BL enable_uart5
	BL enable_usart1

	@ To read in data, we need to use a memory buffer to store the incoming bytes
	@ Get pointers to the buffer and terminating character
	LDR R1, =incoming_buffer
	LDR R2, =user_defined_terminating_character

	@ dereference the terminating character, store it in R2
	LDRB R2, [R2]

	@ Keep a pointer that counts how many bytes have been received
	MOV R3, #0x00

@ continue reading forever (NOTE: eventually it will run out of memory as we don't have a big buffer)
loop_forever:

	LDR R4, =UART5 @ the base address for the register to set up UART
	LDR R0, =USART1
	LDR R5, [R4, USART_ISR] @ load the status of the UART

	TST R5, 1 << UART_ORE | 1 << UART_FE  @ 'AND' the current status with the bit mask that we are interested in
						   @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	BNE clear_error

	TST R5, 1 << UART_RXNE @ 'AND' the current status with the bit mask that we are interested in
							  @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	BEQ loop_forever @ loop back if there was no character to read

	LDRB R6, [R4, USART_RDR] @ load the lowest byte (RDR bits [0:7] for an 8 bit read)

	CMP R2, R6 @ seeing if the termination character has been reached
	BNE no_reset

	STRB R2, [R1, R3]

	B tx_uart

@ When not at termination character, traversing to next character in input string
no_reset:

	STRB R6, [R1, R3] @loading the next character in the string
	ADD R3, #1 @incrementing the offset

	@ load the status of the UART
	LDR R5, [R4, USART_RQR]
	ORR R5, 1 << UART_RXFRQ
	STR R5, [R4, USART_RQR]

	BGT loop_forever

@ Clear the overrun/frame error flag in the register USART_ICR (see page 897)
clear_error:

	LDR R5, [R4, USART_ICR] @ loading the UART clear flag section
	ORR R5, 1 << UART_ORECF | 1 << UART_FECF @ clear the flags (by setting flags to 1)
	STR R5, [R4, USART_ICR] @ storing the cleared flags in the clear flag section
	B loop_forever

@ Transmits the characters from UART to the serial monitor
tx_uart:

	LDR R7, [R0, USART_ISR] @ load the status of the UART
	ANDS R7, 1 << UART_TXE  @ 'AND' the current status with the bit mask that we are interested in
						    @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	@ loop back to check status again if the flag indicates there is no byte waiting
	BEQ tx_uart

	@ load the next value in the string into the transmit buffer for the specified UART
	LDRB R8, [R1], #1

	@ check terminating pointer
	CMP R8, R2
	BEQ loop_forever_reset

	STRB R8, [R0, USART_TDR] @Shifts to the next byte

	@ loop back to the start
	B tx_uart

@ Resets the buffers to allow for process to start again
loop_forever_reset:

	LDR R1, =incoming_buffer
	MOV R3, #0x00
	B loop_forever
