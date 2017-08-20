//**********************************************************************
//
// Simple Serial communicationss with ATmega168.
// Most code is from the datasheet.
//
// electronut.in
//**********************************************************************

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define F_CPU 1000000

//
// BEGIN: serial comms
//
// from data sheet
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr)
{
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  /*Enable receiver and transmitter */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Set frame format: 8data, 1 stop bit */
  UCSR0C = (1<<UCSZ00) | (1 << UCSZ01);
}

void USART_Transmit(unsigned char data )
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  UDR0 = data;
  /* Put data into buffer, sends the data */
}

// write null terminated string
void serial_write_str(const char* str)
{
  int len = strlen(str);
  int i;
  for (i = 0; i < len; i++) {
    USART_Transmit(str[i]);
  }
}

//
// END: serial comms
//

int main (void)
{

  // initialize USART
  USART_Init(9600);

  // loop
	//DDRD=0xFF;
  while (1) {

	  _delay_ms(2000);
      //PORTD = 0xFF;
      //serial_write_str("BOM\n");
      USART_Transmit('s');

  }

  return 1;
}
