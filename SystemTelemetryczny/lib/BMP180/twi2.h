#include <avr/io.h>

#define ACK 1
#define NOACK 0

// procedura transmisji sygnału START
void twistart(void);
// procedura transmisji sygnału STOP
void twistop(void);
// procedura transmisji bajtu danych
void twiwrite(char data);
//procedura odczytu bajtu danych
char twiread(char ack);
