/*
   Bluetooth_Interface with ATmega16 to Control LED via smartphone
   http://www.electronicwings.com

 */

#include <avr/io.h>
#include <util/delay.h>


int main()
{
	DDRB = _BV(1);
	PORTB = _BV(1);
	while(1){
		_delay_ms(500);
		PORTB ^= _BV(1);
	}
	return 0;
}
