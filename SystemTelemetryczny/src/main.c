//**********************************************************************
//
// Simple Serial communicationss with ATmega168.
// Most code is from the datasheet.
//
// electronut.in
//**********************************************************************

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <I2C/i2cmaster.h>
#include <DS085/bmp085.h>
#include <MPU6050/mpu6050.h>

#define F_CPU 2457600UL

#define FOSC 2457600 // Clock Speed
#define BAUD 9600
#define USART0_BAUDRATE FOSC/16/BAUD-1

volatile uint8_t time_1ms;
volatile uint8_t time_10ms;
volatile uint8_t time_1s;



char ocrCountBuff[8];

void USART0_Init(unsigned int ubrr) {
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

void USART0_WRITE_STRING(const char* str) {
	int len = strlen(str);
	int i;
	for (i = 0; i < len; i++) {
		USART_Transmit(str[i]);
	}
}

//wysylanie danych do BT
void wyslijDoHC05(const char* str) {
	switchToBtAtmega(); //wlacz komunikacje ATmega <-> BT
	USART0_WRITE_STRING(str); //wyslanie stringa
	switchToGpsBt(); //wlacz komunikacje GPS -> BT
}
void bmpRead(){
	char itoaTemp[10];

	wyslijDoHC05("\n----------BT180 start--------\n");
	ltoa(bmp085_getpressure(), itoaTemp, 10);
	wyslijDoHC05("Cisnienie->"); wyslijDoHC05(itoaTemp);
	itoa(bmp085_gettemperature(), itoaTemp, 10);
	wyslijDoHC05("\nTemperatura->"); wyslijDoHC05(itoaTemp);
	wyslijDoHC05("\n----------BT180 koniec--------\n");
}
void mpuRead(){
	int16_t axg=0;
	int16_t ayg=0;
	int16_t azg=0;
	int16_t gxds=0;
	int16_t gyds=0;
	int16_t gzds=0;

	char dtostrTemp[15];
	mpu6050_getRawData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);

	wyslijDoHC05("\n-----------MPU6050 start--------\n");

	wyslijDoHC05("\naxg->");
	//dtostrf(axg,8,3, dtostrTemp);
	itoa(axg, dtostrTemp, 10);
	wyslijDoHC05(dtostrTemp);

	wyslijDoHC05("\nayg->");
	//dtostrf(ayg,8,3, dtostrTemp);
	itoa(ayg, dtostrTemp, 10);
	wyslijDoHC05(dtostrTemp);

	wyslijDoHC05("\nazg->");
	//dtostrf(azg,8,3, dtostrTemp);
	itoa(azg, dtostrTemp, 10);
	wyslijDoHC05(dtostrTemp);

	wyslijDoHC05("\ngxds->");
	//dtostrf(gxds,8,3, dtostrTemp);
	itoa(gxds, dtostrTemp, 10);
	wyslijDoHC05(dtostrTemp);

	wyslijDoHC05("\ngyds->");
	//dtostrf(gyds,8,3, dtostrTemp);
	itoa(gyds, dtostrTemp, 10);
	wyslijDoHC05(dtostrTemp);

	wyslijDoHC05("\ngzds->");
	//dtostrf(gzds,8,3, dtostrTemp);
	itoa(gzds, dtostrTemp, 10);
	wyslijDoHC05(dtostrTemp);

	wyslijDoHC05("\n-----------MPU6050 stop--------\n");
}

//wylaczenie wszystkich kluczy na MN4066
void turnOffAllSwitches(){
//TODO wstawic bezpieczniki
	//PORTX &=~(1 << PXX);
	//PORTX &=~(1 << PXX);
	//PORTX &=~(1 << PXX);
	//PORTX &=~(1 << PXX);
}
//wlaczenie kluczy Atmega <-> Bt
void turnOnBtAtmega(){
//TODO wstawic bezpieczniki
	//PORTX|=(1 << PXX);
	//PORTX|=(1 << PXX);
}
//wlaczenie kluczyc Gps -> Bt
void turnOnGpsBt(){
//TODO wstawic bezpieczniki
	//PORTX|=(1 << PXX);
	//PORTX|=(1 << PXX);
}

//komunikacja ATmega <-> BT
void switchToBtAtmega(){
	turnOffAllSwitches();
	turnOnBtAtmega();
}
//komunikacja GPS -> BT
void switchToGpsBt(){
	turnOffAllSwitches();
	turnOnGpsBt();
}
//inicjalizacja wyjsc sterujacych MN4066
void IO_init(){
	//TODO ustawic cztery piny jako out
	turnOffAllSwitches();
}
int main(void) {
	IO_init();
	I2C_INIT();
	USART0_Init(USART0_BAUDRATE);
	bmp085_init();
	mpu6050_init();

	while (1) {
		_delay_ms(2600);
		bmpRead();
		mpuRead();
	}
	return 0;
}
