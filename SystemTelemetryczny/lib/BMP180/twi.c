
#include <BMP180/twi2.h>

// procedura transmisji sygnału START
void twistart(void)
{
TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
while (!(TWCR & (1<<TWINT)));
}

// procedura transmisji sygnału STOP
void twistop(void)
{
TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
while ((TWCR & (1<<TWSTO)));
}

// procedura transmisji bajtu danych
void twiwrite(char data)
{
TWDR = data;
TWCR = (1<<TWINT) | (1<<TWEN);
while (!(TWCR & (1<<TWINT)));
}

//procedura odczytu bajtu danych
char twiread(char ack)
{
TWCR = ack
? ((1 << TWINT) | (1 << TWEN) | (1 << TWEA))
: ((1 << TWINT) | (1 << TWEN)) ;
while (!(TWCR & (1<<TWINT)));
return TWDR;
}
