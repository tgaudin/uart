#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "../uart.h"

int main(void) {
	serial.begin(9600);
	sei();
	serial.write("Hello World!\n\r");
	uint8_t ch;
	while(1) {
		if (serial.available()) {
			ch = serial.read();	
			serial.write(ch);
		}
	}
}

