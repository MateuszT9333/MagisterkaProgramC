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
#include <avr/interrupt.h>
#include <I2C/i2cmaster.h>
#include <DS085/bmp085.h>

#define F_CPU 2457600UL

#define FOSC 2457600 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

volatile uint8_t time_1ms;
volatile uint8_t time_10ms;
volatile uint8_t time_1s;

char ocrCountBuff[8];

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
	//TODO odblokuj kanal UART(zablokuj komunikacje z GPS)
	cli();
	serial_write_str(str); //wyslanie stringa
	sei();
	//TODO zablokuj kanal UART(odblokuj komunikacja dla GPS)
}
void bmpRead(){
	char itoaTemp[10];

	wyslijDoBT("\n----------BT180 start--------\n");
	ltoa(bmp085_getpressure(), itoaTemp, 10);
	wyslijDoBT("Cisnienie->"); wyslijDoBT(itoaTemp);
	itoa(bmp085_gettemperature(), itoaTemp, 10);
	wyslijDoBT("\nTemperatura->"); wyslijDoBT(itoaTemp);
	wyslijDoBT("\n----------BT180 koniec--------\n");
}
void timerInit(){
	TCCR0A |= (1<< WGM01); //CTC
	TCCR0B |= (1<< CS01); //preskaler clk/8
	TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect
	OCR0A= 200;

}
int main(void) {
// Nie używać delay

	timerInit();
	USART_Init(MYUBRR);
	bmp085_init();
	sei();

	while (1) {
		_delay_ms(2600);
		bmpRead();
	}
	return 0;
}

ISR (TIMER0_COMPA_vect)  // timer0 1ms interrupt
{
	if(++time_1ms>9){
			time_1ms=0;

			if(++time_10ms>99){
				time_10ms=0;

				if(++time_1s>59) time_1s=0;
			}
		}

		if(bmp180_time_1ms--);
}
