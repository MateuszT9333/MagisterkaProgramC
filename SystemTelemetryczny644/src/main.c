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

#define FOSC 2457600L // Clock Speed
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
	while (!( UCSR0A & (1 << UDRE0)));
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
void bmpRead(){
	char itoaTemp[10];

	ltoa(bmp085_getpressure(), itoaTemp, 10);
	wyslijDoBT("C"); wyslijDoBT(itoaTemp);
	itoa(bmp085_gettemperature(), itoaTemp, 10);
	wyslijDoBT("T"); wyslijDoBT(itoaTemp);
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


	wyslijDoBT("ax");
	//dtostrf(axg,8,3, dtostrTemp);
	itoa(axg, dtostrTemp, 10);
	wyslijDoBT(dtostrTemp);

	wyslijDoBT("ay");
	//dtostrf(ayg,8,3, dtostrTemp);
	itoa(ayg, dtostrTemp, 10);
	wyslijDoBT(dtostrTemp);

	wyslijDoBT("az");
	//dtostrf(azg,8,3, dtostrTemp);
	itoa(azg, dtostrTemp, 10);
	wyslijDoBT(dtostrTemp);

	wyslijDoBT("gx");
	//dtostrf(gxds,8,3, dtostrTemp);
	itoa(gxds, dtostrTemp, 10);
	wyslijDoBT(dtostrTemp);

	wyslijDoBT("gy");
	//dtostrf(gyds,8,3, dtostrTemp);
	itoa(gyds, dtostrTemp, 10);
	wyslijDoBT(dtostrTemp);

	wyslijDoBT("gz");
	//dtostrf(gzds,8,3, dtostrTemp);
	itoa(gzds, dtostrTemp, 10);
	wyslijDoBT(dtostrTemp);

}

int main(void) {
	i2c_init();
	USART_Init(MYUBRR);
	bmp085_init();
	mpu6050_init();

	while (1) {
		_delay_ms(1000);
		bmpRead();
		mpuRead();
	}
	return 0;
}
