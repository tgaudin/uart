/*
 *	uart.h -- An uart implementation for AVR Atmel Mega 328p (Arduino nano)
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
 *	-> Changelog
 *		25.10.16:	Initial commit.
 *
 *
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/pgmspace.h>

#include "../lib/cbuffer/cbuffer.h"


class uart {
	public:
		uart() = default;
		~uart() = default;
		void begin(const unsigned long baud);
		void end(void);
		bool write(uint8_t c);
		void write(const char *c);
		
		uint8_t read(void);
		bool available(void);
		void _rx_interrupt(void);
		void _tx_interrupt(void);
	private:
		void turn_tx_on(void)  { UCSR0B |=  (1<<UDRIE0); } 
		void turn_tx_off(void) { UCSR0B &= ~(1<<UDRIE0); }
		CBuffer<uint8_t, 32> rx_buffer;
		CBuffer<uint8_t, 32> tx_buffer;
};

extern uart serial;

#endif
