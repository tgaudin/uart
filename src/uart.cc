/*
 *	uart.cc -- An uart implementation for AVR Atmel Mega 328p (Arduino nano)
 *
 *	-> Author:		Théophile Gaudin
 *	-> Version:		1.0
 *	-> Date:		25.10.2016
 *
 * 
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <theophile.gaudin@gmail.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return. Théophile Gaudin
 *
 */
 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../uart.h"
#include "../lib/cbuffer/cbuffer.h"

//uart *uart::ptr = 0;

void uart::begin(const unsigned long baud) {
	// u2x mode 
	UCSR0A |= (1<<U2X0);
	uint16_t ubrr = (F_CPU / (8*baud)) - 1;
	
	// Setting the baud rate
	//unsigned long ubrr = ((((F_CPU/16) + (baud/2)) / (baud)) -1);
 
	UBRR0H = (uint8_t) (ubrr >> 8);
	UBRR0L = (uint8_t) ubrr;
	// Enabling RX and TX
	UCSR0B |= (1<<RXEN0) | (1<<TXEN0);
	// Enabling RXC interrupt
	UCSR0B |= (1 << RXCIE0);
	// Frame format, 8 data bits, 1 stop bit
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01) | (1<<USBS0);
}

void uart::end(void) {
	UCSR0B &= ~(1<<RXEN0) | ~(1<<TXEN0);
	turn_tx_off();
	UCSR0B &= ~(1<<RXCIE0);
}

bool uart::write(uint8_t ch) {
	if (tx_buffer.is_full()) return false;
	// If UDR0 is empty and the buffer is empty as well
	// writing directly the data into UDR0
	// It's more efficient to do that
	if (( UCSR0A & (1<<UDRE0) ) && ( tx_buffer.is_empty() )) {
		UDR0 = ch;
	} else {
		tx_buffer.write(&ch);
		// Activating transmission
		turn_tx_on();
	}
	return true;
}

void uart::write(const char *c) {
	while (*c) write(*c++);
}

uint8_t uart::read(void) {
	bool status;
	uint8_t data;
	status = rx_buffer.read(&data);
	if (!status) return -1;
	return data;
}

bool uart::available(void) {
	if (rx_buffer.is_empty()) return false;
	return true;
}


void uart::_rx_interrupt(void) {
	uint8_t data = UDR0;
	rx_buffer.write(&data);
}

void uart::_tx_interrupt(void) {
	if( tx_buffer.is_empty() ) {
		turn_tx_off();
	} else {
		uint8_t data; 
		tx_buffer.read(&data);
		UDR0 = data;
	}
}

uart serial;

ISR(USART_RX_vect) {
	serial._rx_interrupt();
}

ISR(USART_UDRE_vect) {
	serial._tx_interrupt();
}

