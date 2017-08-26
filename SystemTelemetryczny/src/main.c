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
#define F_CPU 2457600UL


#define FOSC 2457600 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

volatile uint8_t time_1ms;
volatile uint8_t time_10ms;
volatile uint8_t time_1s;


// procedura transmisji sygnału START
void twiStart(void) {

	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)))
		;
}

// procedura transmisji sygnału STOP
void twiStop(void) {

	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	while ((TWCR & (1 << TWSTO)))
		;
}

// procedura transmisji bajtu danych
void twiWrite(uint8_t data) {

	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)))
		;
}

//procedura odczytu bajtu danych
uint8_t twiRead(uint8_t ack) {

	TWCR = ack ?
			((1 << TWINT) | (1 << TWEN) | (1 << TWEA)) :
			((1 << TWINT) | (1 << TWEN));
	while (!(TWCR & (1 << TWINT)))
		;
	return TWDR;
}

//
// BEGIN: serial comms
//
// from data sheet

volatile uint16_t ocrCount = 0;
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
	cli();
	serial_write_str(str); //wyslanie stringa
	sei();
}
void timerInit(){
	TCCR0A |= (1<< WGM01); //CTC
	TCCR0B |= (1<< CS01); //preskaler clk/8
	TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect
	OCR0A= 200;

}
void BMPInit(){

	void (*TWI_start)(void) = twiStart;
	void (*TWI_stop)(void) = twiStop;
	uint8_t (*TWI_read)(uint8_t) = twiRead;
	void (*TWI_write)(uint8_t) = twiWrite;

	register_twi_start_bmp180(TWI_start);						//rejestracja funkcji start z i2c
	register_twi_stop_bmp180(TWI_stop);							//rejestracja funkcji stop z i2c
	register_twi_read_bmp180(TWI_read);							//rejestracja funkcji read z i2c
	register_twi_write_bmp180(TWI_write);

	ds_bmp180_coeff();
	ds_bmp180_start(BMP180_OSS_8);

}
void bmpMeasure(){
	static int time_ls;
	char itoaTemp [30];
	if( (time_1s != time_ls) && !(time_1s % 2) ){
		wyslijDoBT("\n----------BMP180 start------------");
		time_ls = time_1s;

		ds_bmp180_start(BMP180_OSS_8);


		itoa(ds_bmp180.bmp180_temp_sign, itoaTemp, 10);
		wyslijDoBT("\nznak: ");
		wyslijDoBT(itoaTemp);

		itoa(ds_bmp180.bmp180_temp_cell, itoaTemp, 10);
		wyslijDoBT("\ntempc: ");
		wyslijDoBT(itoaTemp);

		itoa(ds_bmp180.bmp180_temp_frac, itoaTemp, 10);
		wyslijDoBT("\ntempp: ");
		wyslijDoBT(itoaTemp);

		itoa(ds_bmp180.bmp180_pressure_cell, itoaTemp, 10);
		wyslijDoBT("\ncisnieniec: ");
		wyslijDoBT(itoaTemp);

		itoa(ds_bmp180.bmp180_pressure_frac, itoaTemp, 10);
		wyslijDoBT("\ncisnieniep: ");
		wyslijDoBT(itoaTemp);
		wyslijDoBT("\n----------BMP180 stop------------");

	}

	ds_bmp180_measure();
}
int main(void) {
// Nie używać delay;
	timerInit();
	USART_Init(MYUBRR);
	BMPInit();
	sei();

	while (1) {

		bmpMeasure();
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
