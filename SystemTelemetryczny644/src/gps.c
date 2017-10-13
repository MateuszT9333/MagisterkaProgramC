
#include <avr/interrupt.h>
#include <avr/iomxx4.h>
#include <DS085/bmp085.h>
#include <GPS/gps.h>
#include <I2C/i2cmaster.h>
#include <MPU6050/mpu6050.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#define FOSC 8000000L // Clock Speed
#define BAUD0 9600
#define BAUD1 9600
#define USART0_BAUDRATE (FOSC / 4 / BAUD0 - 1) / 2
#define USART1_BAUDRATE (FOSC / 4 / BAUD1 - 1) / 2


void USART0_Init(unsigned int ubrr) { //inicjalizacja Bluetooth
	/*Set baud rate */
	cli();
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	// double speed operation
	UCSR0A = (1<<U2X1);
	/*Enable receiver and transmitter */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	/* Set frame format: 8data, 1 stop bit */
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
	sei();
}
void USART1_Init(unsigned int ubrr) { //inicjalizacja GPS
	UBRR1H = (unsigned char) (ubrr >> 8);
	UBRR1L = (unsigned char) ubrr;
	UCSR1A = (1<<U2X1);							//Double speed operation
	UCSR1B = (1<<RXEN1) | (1 << TXEN0);		//Enable only RX
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);		//8 bit data
}

void USART0_Transmit(unsigned char data) {
	/* Wait for empty transmit buffer */
	while (!( UCSR0A & (1 << UDRE0)));
	UDR0 = data;
	/* Put data into buffer, sends the data */
}
unsigned char USART0_Receive( void ){

	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

void USART0_WRITE_STRING(const char* str) {
	int len = strlen(str);
	int i;
	for (i = 0; i < len; i++) {
		USART0_Transmit(str[i]);
	}
}
void USART1_Transmit(unsigned char data) {
	/* Wait for empty transmit buffer */
	while (!( UCSR1A & (1 << UDRE1)));
	UDR1 = data;
	/* Put data into buffer, sends the data */
}
unsigned char USART1_Receive(void) {
	/* Wait for data to be received */
	while (!(UCSR1A & (1 << RXC1)));
	wdt_reset();
	/* Get and return received data from buffer */
	return UDR1;
}
// write null terminated string

void USART1_WRITE_STRING(const char* str) {
	int len = strlen(str);
	int i;
	for (i = 0; i < len; i++) {
		USART1_Transmit(str[i]);
	}
}

//wysylanie danych do BT
void sendToHC05(const char* str) {
	USART0_WRITE_STRING(str); //wyslanie stringa
}
//wysylanie danych do GPS
void sendToGPS(const char* str) {
	USART1_WRITE_STRING(str); //wyslanie stringa
}

void bmpRead(){
	char itoaTemp[10];

	ltoa(bmp085_getpressure(), itoaTemp, 10);
	sendToHC05("C"); sendToHC05(itoaTemp);
	sendToHC05("\n");

	itoa(bmp085_gettemperature(), itoaTemp, 10);
	sendToHC05("T"); sendToHC05(itoaTemp);
	sendToHC05("\n");

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


	sendToHC05("ax");
	//dtostrf(axg,8,3, dtostrTemp);
	itoa(axg, dtostrTemp, 10);
	sendToHC05(dtostrTemp);
	sendToHC05("\n");

	sendToHC05("ay");
	//dtostrf(ayg,8,3, dtostrTemp);
	itoa(ayg, dtostrTemp, 10);
	sendToHC05(dtostrTemp);
	sendToHC05("\n");

	sendToHC05("az");
	//dtostrf(azg,8,3, dtostrTemp);
	itoa(azg, dtostrTemp, 10);
	sendToHC05(dtostrTemp);
	sendToHC05("\n");

	sendToHC05("gx");
	//dtostrf(gxds,8,3, dtostrTemp);
	itoa(gxds, dtostrTemp, 10);
	sendToHC05(dtostrTemp);
	sendToHC05("\n");

	sendToHC05("gy");
	//dtostrf(gyds,8,3, dtostrTemp);
	itoa(gyds, dtostrTemp, 10);
	sendToHC05(dtostrTemp);
	sendToHC05("\n");

	sendToHC05("gz");
	//dtostrf(gzds,8,3, dtostrTemp);
	itoa(gzds, dtostrTemp, 10);
	sendToHC05(dtostrTemp);
	sendToHC05("\n");
}

void GPS_Send_PMTK(){
	sendToGPS("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"); //tylko gprmc
}
void GPS_Simple_Receive() {
	unsigned char value = "";
	uint8_t odbiorGPRMC = 0;
	while (odbiorGPRMC == 0) {
		value = USART1_Receive();

		if (value == '$') {
			value = USART1_Receive();
			if (value == 'G') {
				value = USART1_Receive();

				if (value == 'P') {
					value = USART1_Receive();

					if (value == 'R') {
						value = USART1_Receive();

						if (value == 'M') {
							value = USART1_Receive();

							if (value == 'C') { // mamy gprmc
								while (value != '\n') {
									value = USART1_Receive();
									USART0_Transmit(value);
								}

								odbiorGPRMC = 1;
							}
						}

					}
				}
			}
		}
	}
}
void voltageOnBattery() {
	char temp[15];
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	itoa(ADC, temp, 10);
	sendToHC05("V");
	sendToHC05(temp);
	sendToHC05("\nEND\n");
	_delay_ms(10);
	sleepForXInterrupts(15);
}
void sleepForXInterrupts(int numOfInterrupts) {
	TCCR2B |= (1<<CS22) | (1<<CS21)|(1<<CS20);
	TIMSK2 |= (1<<TOIE2);
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	sleep_enable();
	for(int i = numOfInterrupts; i>0; i--){
		sleep_mode();
	}
	TCCR2B = 0;
	TIMSK2 = 0;

}
void ADCEnable(){
	ADCSRA |= (1<<ADEN);
}

/**
	@brief Start point for the application
*/
int main(void) {
	wdt_enable(WDTO_8S);
	ADCEnable();
	USART1_Init(USART1_BAUDRATE);
	USART0_Init(USART0_BAUDRATE);
	i2c_init();
	bmp085_init();
	mpu6050_init();
	while (1) {
		bmpRead();
		mpuRead();
		voltageOnBattery();
		GPS_Simple_Receive();
	}
}
ISR(TIMER2_OVF_vect){
	sleep_disable();
}

