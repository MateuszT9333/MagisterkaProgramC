/**
	@date 2013/07/08

	@brief GPS with the Teensy 2.0 micro-controller

	An application to read from an <a href='http://www.gtop-tech.com/en/product/MT3339_GPS_Module_04.html'>GlobalTop PA6H GPS module</a>.
	Supports NEMA parsing, and uses dynamic memory allocation and linked lists to provide a reliable state machine.

	Here's how it works;
		Init all the vars, start up the USART and wait for input.

		When the GPS module sends a character, the interrupt for USART rx is called,
		our code puts the character into a buffer (of size NMEA_BUFFER_LEN) and
		checks to see if it was a newline? (this indicates the end of a NMEA sentance).
		If it is, we now have a full NMEA sentance in our buffer, if not, carry on till we do
 		(and don't overflow).

		Now we create a nmeaData instance in memory, set the nmeaString pointer to our buffer,
		and make ourselves a new buffer instance.

		Next we update the 'next' attribute of the nmeaData the main loop is using (gpsData) to
		point at our new sentence. This triggers the main loop into processing the previous item.

		We move gpsData along to the new instance, process and free the old instance, and wait to
		start again.

		In theory we should never miss a sentance as they're all buffered, though if we're too slow
		reading the nmeaData items we'll fill the memory and crash...

	Notes:
		If at any point the LED on pin 6 lights up, a malloc() has failed in the ISR and our
		memory is full, we're probably crashing at this point - at least we know why and can debug.

		We could move the filter to be inside the USART rx interrupt to save on malloc'd memory
		but it seems fine and really we want to keep the ISR as small (quick) as possible. It's
		probably best this way round.

		I'm testing on a GlobalTop PA6H GPS module (using a MediaTek MT3339 chipset) which supports
		selective output, so you could easily filter unused NMEA sentences on the gps side, removing
		the need to filter on the mcu side. Enable only GPRMC in this case by sending:
			$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29
		to the unit. To re-enable all NMEA strings send:
			$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28

		Update rate can also be changed in a similar way.

	Connections:
		+5v to +5v, GND to GND, TX on GPS to PD2(RX)
*/

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

///Serial baudrate config

///Maximum length of a NEMA string for buffer allocation
#define NMEA_BUFFER_LEN 85

#define FOSC 8000000L // Clock Speed
#define BAUD0 9600
#define BAUD1 9600
#define USART0_BAUDRATE (FOSC / 4 / BAUD0 - 1) / 2
#define USART1_BAUDRATE (FOSC / 4 / BAUD1 - 1) / 2

/*
		Globals
*/
///Pointer to our current RX buffer
volatile char *buffer;

///Counter for our current position in the buffer.
///We don't check that (buffer_index < bufferSize) because we know the NEMA limits
volatile uint8_t buffer_index = 0;

///Pointer to our linked list of NEMA strings
nmeaData *gpsData;
unsigned char value;

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
	cli();
	UBRR1H = (unsigned char) (ubrr >> 8);
	UBRR1L = (unsigned char) ubrr;
	UCSR1A = (1<<U2X1);							//Double speed operation
	UCSR1B = (1<<RXEN1) | (1 << TXEN0);		//Enable only RX
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);		//8 bit data
	sei();
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
	while (!(UCSR1A & (1 << RXC1)))
		;
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
	sendToHC05("\nEND\n");

}

void GPS_Init(){
	//Init linked list, global buffer
		gpsData = malloc(sizeof(nmeaData));
		gpsData->next = 0;
		buffer = malloc(sizeof(char)*NMEA_BUFFER_LEN);

}
void GPS_Send_PMTK(){
	sendToGPS("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"); //tylko gprmc
}

/**
	@brief Start point for the application
*/
int main(void) {

	//GPS_Init();
	USART1_Init(USART1_BAUDRATE);
	USART0_Init(USART0_BAUDRATE);
	i2c_init();
	bmp085_init();
	mpu6050_init();
	GPS_Send_PMTK();

	while (1) {
		//value = USART1_Receive();
		USART0_Transmit(USART1_Receive());
//		if (value == '$') {
//			value = USART1_Receive();
//
//			if (value == 'G') {
//				value = USART1_Receive();
//
//				if (value == 'P') {
//					value = USART1_Receive();
//
//					if (value == 'R') {
//						value = USART1_Receive();
//
//						if (value == 'M') {
//							value = USART1_Receive();
//
//							if (value == 'C') {
//							}
//						}
//					}
//				}
//			}
//		}
	}
}


