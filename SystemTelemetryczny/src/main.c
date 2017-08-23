//**********************************************************************
//
// Simple Serial communicationss with ATmega168.
// Most code is from the datasheet.
//
// electronut.in
//**********************************************************************

#include <avr/io.h>
#include <util/delay.h>
#include <BMP180/ds_bmp180.h>
s
#define F_CPU 2457600UL


#define FOSC 2457600 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

//
// BEGIN: serial comms
//
// from data sheet

void USART_Init(unsigned int ubrr) {
	/*Set baud rate */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	/* Set frame format: 8data, 1 stop bit */
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}

void USART_Transmit(unsigned char data) {
	/* Wait for empty transmit buffer */
	while (!( UCSR0A & (1 << UDRE0)))
		;
	UDR0 = data;
	/* Put data into buffer, sends the data */
}

// write null terminated string
void serial_write_str(const char* str) {
	int len = strlen(str);
	int i;
	for (i = 0; i < len; i++) {
		USART_Transmit(str[i]);
	}
}

//wysylanie danych do BT
void wyslijDoBT(const char* str) {

	serial_write_str(str); //wyslanie stringa
}

int main(void) {
	long int press;
	long int temp;



	while (1) {
		_delay_ms(2600); //TODO nieprawidlowa wartosc opoznienia
		wyslijDoBT("OK!\n");
	}

	return 0;
}
