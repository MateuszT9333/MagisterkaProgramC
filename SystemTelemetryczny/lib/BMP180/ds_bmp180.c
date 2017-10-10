/*
 * ds_bmp180.c
 *
 *  Created on: 21-12-2015
 *	 Edited on: 29-12-2015
 *      Author: Daniel
 *		   web: www.danielo88.cba.pl
 *	   version: v1.0
 *
 *	All rights reserved.
 *
 *
 *	Przy pomocy ksi��ki:
 *							Mikrokontrolery AVR
 *							J�zyk C - podstawy programowania
 *
 *															Autor:
 *																Miros�aw Karda�
 *							J�zyk C.
 *							Pasja programowania mikrokontroler�w 8-bitowych
 *
 *															Autor:
 *																Miros�aw Karda�
 *
 *	Do u�ytkownika dost�pne s� nast�puj�ce elementy:
 *						*****funkcje do obs�ugi hp02s
 *
 * 		uint8_t ds_bmp180_coeff(void);											//odczyt wsp�czynnik�w
 *	 	uint8_t ds_bmp180_measure(void);										//pomiar cisnienia - zwraca kt�ry etap jest wykonywany w danej chwili - 0 dla braku wykonywanego pomiaru
 *	 																																				  - 1 dla startu pomiaru
 *	 																																				  - 2 po odczytaniu D1
 *	 																																				  - 3 po odczytaniu T1
 *	 																																				  - 4 koniec pomiaru
 *	 																																				  - 5 nast�pny pomiar do usrednienia
 * 		void ds_bmp180_start(uint8_t pres_oversampling);						//start pomiaru i ustawienie oversampling
 * 																															-	BMP180_OSS_1	0
 * 																															-	BMP180_OSS_2	1
 * 																															-	BMP180_OSS_4	2
 * 																															-	BMP180_OSS_8	3
 * 		void ds_bmp180_restart(void);											//restart uk�adu
 * 		uint8_t ds_bmp180_active(void);											//sprawdzenie czy uk�ad jest aktywny
 *
 *
 *						*****rejestracja funkcji i2c potrzebnych do prawid�owego dzia�ania biblioteki
 *
 *		void register_twi_start_bmp180(void (*twi_callback)(void));				//rejestracja funkcji start z i2c
 * 		void register_twi_stop_bmp180(void (*twi_callback)(void));				//rejestracja funkcji stop z i2c
 * 		void register_twi_write_bmp180(void (*twi_callback)(uint8_t bajt));		//rejestracja funkcji zapisuj�cej do i2c
 * 		void register_twi_read_bmp180(uint8_t (*twi_callback)(uint8_t ack));	//rejestracja funkcji odczytuj�cej z i2c
 *
 *
 *	W strukturze dost�pnej dla u�ytkownika znajduje si� nast�puj�ce dane:
 *
 *		int32_t	bmp180_UT;					//wartosc UT odczytana z uk�adu
 *		int32_t bmp180_UP;					//wartosc UP odczytana z uk�adu
 *		uint8_t bmp180_temp_sign;			//temperatura - znak
 *		uint8_t bmp180_temp_cell;			//temperatura - calosc
 *		uint8_t bmp180_temp_frac;			//temperatura - wartosc po przecinku
 *		uint16_t bmp180_pressure_cell;		//cisnienie - calosc
 *		uint8_t bmp180_pressure_frac;		//cisnienie - wartosc po przecinku
 *		uint8_t chip_id;					//id chipu - 0x55
 *		uint8_t ver_regist;					//wersja rejestru
 *
 *
 *	UWAGA!!!
 *
 *	W tym pliku nie nale�y niczego zmienia�
 *	Do prawid�owego dzia�ania biblioteki, konieczne jest umieszczenie warunku:
 *																				if(bmp180_time_1ms--);
 *																										w przerwaniu wyst�puj�cym co 1ms.
 *
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ds_bmp180.h"

//adres uk�adu bmp180
#define	BMP180_ADR			0xEE

//adres rejestru kontrolnego
#define	BMP180_REG_ADR_CTRL		0xF4
#define	BMP180_REG_MEAS_TEMP	0X2E	//pomiar temperatury
#define BMP180_REG_MEAS_PRES_1	0x34	//pomiar cisnienia - oversampling pojedynczy
#define BMP180_REG_MEAS_PRES_2	0x74	//pomiar cisnienia - oversampling podw�jny
#define BMP180_REG_MEAS_PRES_4	0xB4	//pomiar cisnienia - oversampling poczw�rny
#define BMP180_REG_MEAS_PRES_8	0xF4	//pomiar cisnienia - oversampling osmiokrotny

#define BMP180_REG_CTRL_OSS		0x34	//pomiar cisnienia - baza do ustawienia oversamplingu

#define BMP180_START_CONVERSION	5		//bit w rejestrze 0xF4 kt�ry okresla start pomiaru

//adres kom�rki przechowuj�cej wynik pomiaru
#define BMP180_VALUE_ADR_MSB	0xF6
#define BMP180_VALUE_ADR_LSB	0xF5
#define BMP180_value_ADR_XLSB	0xF8	//odczyt najmniej znaczacych bitow dla UP - zalezy od OSS

//adres kom�rki przechowuj�cej id uk�adu
#define BMP180_ADR_ID			0xD0

//adres kom�rki przechowuj�cej wersj� rejestru
#define	BMP180_ADR_REG_VER		0xD1

//adres pierwszej kom�rki w pami�ci w kt�rej znajduje si� 8 najbardziej znacz�cych bit�w pierwszego wsp�czynnika AC1
#define BMP180_ADR_COEF_AC1		0xAA

//reset programowy uk�adu bmp180
#define BMP180_RST_SOFT_REG		0xE0		//adres kom�rki
#define BMP180_RST_SOFT			0xB6		//wartosc


//*****	zmienne potrzebne do obs�ugi bmp180 widoczne na zewn�trz

DS_BMP180	ds_bmp180;
volatile uint8_t bmp180_time_1ms;

//*****	struktury i zmienne potrzebne do obs�ugi bmp180 widoczne tylko wewn�trz
typedef struct{
	int16_t		coef_AC1;
	int16_t		coef_AC2;
	int16_t		coef_AC3;
	uint16_t	coef_AC4;
	uint16_t	coef_AC5;
	uint16_t	coef_AC6;
	int16_t		coef_B1;
	int16_t		coef_B2;
	int16_t		coef_MB;
	int16_t		coef_MC;
	int16_t		coef_MD;
}BMP180_COEF;

typedef union{
	uint8_t	status;
	struct{
		uint8_t		bmp180_start:1;			//0
		uint8_t		bmp180_meas_temp:1;		//1
		uint8_t		bmp180_meas_pres:1;		//2
		uint8_t		bmp180_end_meas:1;		//3
		uint8_t		bmp180_active:1;		//4		- czy uk�ad jest aktywny i dziala
		uint8_t		bmp180_pres_oss:2;		//5,6
	};
}BMP180_STATUS_CHIP;

static volatile BMP180_COEF bmp180_value_callibration;
static volatile BMP180_STATUS_CHIP bmp180_status;
static int32_t bmp_x1=0, bmp_x2=0,  bmp_b5=0, bmp_x3=0, bmp_b6=0;
static uint32_t bmp_b4=0, bmp_b7=0;


//*****	funkcje widoczne tylko wewn�trz
static void (*bmp180_twi_start)(void);
static void (*bmp180_twi_stop)(void);
static void (*bmp180_twi_write)(uint8_t bajt);
static uint8_t (*bmp180_twi_read)(uint8_t bajt);


//temperatura
void bmp180_calculate_temperatur(void){
	//funkcja obliczajaca temperatur�

	bmp_x1 = ds_bmp180.bmp180_UT - bmp180_value_callibration.coef_AC6;
	bmp_x1 = bmp_x1 * bmp180_value_callibration.coef_AC5;
	bmp_x1 = bmp_x1 >> 15;

	bmp_x2 = ( (int32_t)bmp180_value_callibration.coef_MC << 11) / (bmp_x1 + bmp180_value_callibration.coef_MD);

	bmp_b5 = bmp_x1 + bmp_x2;

	bmp_x1 = bmp_b5 + 8;
	bmp_x1 = bmp_x1 >> 4;

	ds_bmp180.bmp180_temp_cell = bmp_x1/10;

	if(bmp_x1 < 0){
		ds_bmp180.bmp180_temp_cell = ~(bmp_x1/10) + 1;
		ds_bmp180.bmp180_temp_frac = ~(bmp_x1%10) + 1;
		ds_bmp180.bmp180_temp_sign = 1;
	}else{
		ds_bmp180.bmp180_temp_cell = bmp_x1/10;
		ds_bmp180.bmp180_temp_frac = bmp_x1%10;
		ds_bmp180.bmp180_temp_sign = 0;
	}


}

//cisnienie
void bmp180_calculate_pressure(void){
	//funkcja obliczaj�ca cisnienie

	bmp_b6 = bmp_b5 - 4000;

	bmp_x1 = ( bmp180_value_callibration.coef_B2 * ( (bmp_b6 * bmp_b6) >> 12)) >> 11;
	bmp_x2 = ( bmp180_value_callibration.coef_AC2 * bmp_b6 ) >> 11;
	bmp_x3 = bmp_x1 + bmp_x2;

	bmp_b5 = ( ( ( (int32_t)bmp180_value_callibration.coef_AC1 * 4 + bmp_x3) << bmp180_status.bmp180_pres_oss ) + 2 ) / 4 ;//we wzorze jest b3, b5 zastosowane ze wzgledu na pamiec

	bmp_x1 = ( bmp180_value_callibration.coef_AC3 * bmp_b6 ) >> 13;
	bmp_x2 = ( bmp180_value_callibration.coef_B1 * ( (bmp_b6 * bmp_b6) >> 12) ) >> 16;
	bmp_x3 = ( (bmp_x1 + bmp_x2) + 2) >> 2;

	bmp_b4 = ( bmp180_value_callibration.coef_AC4 * (uint32_t)(bmp_x3 + 32768)) >> 15;

	bmp_b7 = ( (uint32_t)ds_bmp180.bmp180_UP - bmp_b5) * (50000 >> bmp180_status.bmp180_pres_oss);

	if(bmp_b7 < 0x80000000){
		bmp_x3 = (bmp_b7 * 2) / bmp_b4;		//zamiast p
	}else{
		bmp_x3 = (bmp_b7 / bmp_b4) * 2;		//zamiast p
	}

	bmp_x1 = ( bmp_x3 >> 8 ) * (bmp_x3 >> 8);

	bmp_x1 = (bmp_x1 * 3038) >> 16;
	bmp_x2 = (-7357 * bmp_x3) >>16;

	bmp_x3 = bmp_x3 + ( (bmp_x1 + bmp_x2 + 3791) >> 4);

	ds_bmp180.bmp180_pressure_cell = bmp_x3/100;
	ds_bmp180.bmp180_pressure_frac = bmp_x3%100;
}


//*****	funkcje widoczne na zewn�trz

uint8_t ds_bmp180_coeff(void){
	//odczyt wsp�czynnik�w kalibruj�cych uk�ad, oraz danych identyfikuj�cych uk�ad

	//sprawdzenie pod�aczenia uk�adu
	ds_bmp180_active();

	if(bmp180_status.bmp180_active){

		bmp180_status.bmp180_active = 1;

		bmp180_twi_start();
		bmp180_twi_write(BMP180_ADR);				//adres uk�adu do zapisu
		bmp180_twi_write(BMP180_ADR_COEF_AC1);		//adres pierwszej kom�rki w pamieci eeprom
		bmp180_twi_start();							//ponowny start
		bmp180_twi_write(BMP180_ADR | 0x01);		//adres z �adaniem odczytu
		bmp180_value_callibration.coef_AC1 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_AC2 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_AC3 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_AC4 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_AC5 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_AC6 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_B1 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_B2 = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_MB = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_MC = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(1) );
		bmp180_value_callibration.coef_MD = (bmp180_twi_read(1) << 8) | ( bmp180_twi_read(0) );
		bmp180_twi_stop();

		bmp180_twi_start();
		bmp180_twi_write(BMP180_ADR);				//adres uk�adu do zapisu
		bmp180_twi_write(BMP180_ADR_REG_VER);			//odczytanie id _chip
		bmp180_twi_start();
		bmp180_twi_write(BMP180_ADR | 0x01);
		ds_bmp180.ver_regist = bmp180_twi_read(0);
		bmp180_twi_stop();

		return 1;

	}else{
		return 0;
	}

}

uint8_t ds_bmp180_measure(void){
	//odczyt wartosci zmierzonych przez czujnik
	static uint8_t bmp180_step = 0;		//kt�ry krok jest w chwili obecnej

	if( (bmp180_status.bmp180_start) && (bmp180_status.bmp180_active) ){

		switch(bmp180_step){
			case 0:
				//rozpoczecie pomiaru temperatury
				bmp180_twi_start();
				bmp180_twi_write(BMP180_ADR);				//adres urz�dzenia
				bmp180_twi_write(BMP180_REG_ADR_CTRL);		//adres rejestru
				bmp180_twi_write(BMP180_REG_MEAS_TEMP);		//pomiar temperatury
				bmp180_twi_stop();

				bmp180_time_1ms = 6;						//odczekanie 5ms na wykonanie pomiaru
				bmp180_step = 1;							//przejscie od nast�nego kroku
			break;
			case 1:
				//odczekanie 5ms
				if(bmp180_time_1ms){		//jezeli jeszcze liczy
					break;
				}else{						//jezeli min�o 5ms
					bmp180_step = 2;		//przejscie do nast�pnego kroku
				}
			break;
			case 2:
				//Odczyt temperautry z rejestr�w czujnika
				bmp180_twi_start();
				bmp180_twi_write(BMP180_ADR);				//adres czujnika
				bmp180_twi_write(BMP180_VALUE_ADR_MSB);		//adres rejestru zawierajacego zmierzone wartosci
				bmp180_twi_start();
				bmp180_twi_write(BMP180_ADR | 0x01);
				ds_bmp180.bmp180_UT = ( bmp180_twi_read(1) << 8) | ( bmp180_twi_read(0) );
				bmp180_twi_stop();

				bmp180_step = 3;							//przejscie do nast�pnego kroku
			break;
			case 3:
				//rozpocz�cie pomiaru cisnienia
				bmp180_twi_start();
				bmp180_twi_write(BMP180_ADR);				//adres urz�dzenia
				bmp180_twi_write(BMP180_REG_ADR_CTRL);		//adres rejestru
				bmp180_twi_write( (bmp180_status.bmp180_pres_oss << 6) | BMP180_REG_CTRL_OSS);		//pomiar cisnienia
				bmp180_twi_stop();

				bmp180_time_1ms = 6 + 7 * bmp180_status.bmp180_pres_oss;						//odczekanie 5ms na wykonanie pomiaru
				bmp180_step = 4;							//przejscie od nast�nego kroku
			break;
			case 4:
				//odczekanie okreslonego czasu
				if(bmp180_time_1ms){		//jezeli jeszcze liczy
					break;
				}else{						//jezeli min�o 5ms
					bmp180_step = 5;		//przejscie do nast�pnego kroku
				}
			break;
			case 5:
				//Odczyt cisnienia z rejestr�w czujnika
				bmp180_twi_start();
				bmp180_twi_write(BMP180_ADR);				//adres czujnika
				bmp180_twi_write(BMP180_VALUE_ADR_MSB);		//adres rejestru zawierajacego zmierzone wartosci
				bmp180_twi_start();
				bmp180_twi_write(BMP180_ADR | 0x01);
				ds_bmp180.bmp180_UP =  ( ( (uint32_t)bmp180_twi_read(1) << 16)  |  ( (uint16_t)bmp180_twi_read(1) ) << 8 | ( bmp180_twi_read(0) ) ) >> ( 8 - bmp180_status.bmp180_pres_oss);// |

				bmp180_twi_stop();

				bmp180_step = 6;							//przejscie do nast�pnego kroku

				break;

			case 6:
				//obliczenie temperatury
				bmp180_calculate_temperatur();
				bmp180_status.bmp180_meas_temp = 1;			//pomiar i obliczenie temperatury zosta� wykonany
				bmp180_step = 7;
				break;

			case 7:
				//obliczenie cisnienia
				bmp180_calculate_pressure();
				bmp180_status.bmp180_meas_pres = 1;			//pomiar i obliczenie cisnienia zosta� wykonany
				bmp180_step = 8;
				break;

			case 8:
				//pomiar i odczyt zosta� wykonany
				bmp180_status.bmp180_end_meas = 1;
				bmp180_step = 9;
				break;

			case 9:
				//wyzerowanie zmiennych pomocniczych
				bmp180_status.bmp180_start = 0;
				bmp180_status.bmp180_end_meas = 0;
				bmp180_step = 0;
				break;
		}

	}



	return bmp180_status.bmp180_end_meas;
}

void ds_bmp180_start(uint8_t pres_oversampling){
	/* Start wykonania pomiaru
	 * Oversampling BMP180_OSS_1/2/4/8
	 */

	if( (!bmp180_status.bmp180_start) && (bmp180_status.bmp180_active) ){	//jezeli poprzedni pomiar zostal wykonany

		bmp180_status.bmp180_pres_oss = pres_oversampling;
		bmp180_status.bmp180_start = 1;

	}


}

void ds_bmp180_restart(void){

	/*
	 * Programowy reset uk�adu
	 * Taki sam efekt jak po wylaczeniu zasilania
	 *
	 */

	bmp180_twi_start();
	bmp180_twi_write(BMP180_ADR);				//adres uk�adu do zapisu
	bmp180_twi_write(BMP180_RST_SOFT_REG);		//rejestr powoduj�cy reset uk�adu
	bmp180_twi_write(BMP180_RST_SOFT);			//wartosc powodujaca reset ukladu
	bmp180_twi_stop();


}

uint8_t ds_bmp180_active(void){
	/*
	 * Funkcja sprawdzaj�ca czy uk�ad jest aktywny i czy dziala
	 */

	//sprawdzenie pod�aczenia uk�adu
	bmp180_twi_start();
	bmp180_twi_write(BMP180_ADR);				//adres uk�adu do zapisu
	bmp180_twi_write(BMP180_ADR_ID);			//odczytanie id _chip
	bmp180_twi_start();
	bmp180_twi_write(BMP180_ADR | 0x01);
	ds_bmp180.chip_id = bmp180_twi_read(0);
	bmp180_twi_stop();

	if(ds_bmp180.chip_id == 0x55){
		bmp180_status.bmp180_active = 1;
		return 1;
	}else{

		bmp180_status.bmp180_active = 0;
		return 0;
	}
}



//*****	rejestracja funkcji i2c potrzebnych do prawid�owego dzia�ania biblioteki

void register_twi_start_bmp180(void (*twi_callback)(void)){
	//rejestracja funkcji start z i2c
	bmp180_twi_start = twi_callback;
}

void register_twi_stop_bmp180(void (*twi_callback)(void)){
	//rejestracja funkcji stop z i2c
	bmp180_twi_stop = twi_callback;
}

void register_twi_write_bmp180(void (*twi_callback)(uint8_t bajt)){
	//rejestracja funkcji zapisuj�cej do i2c
	bmp180_twi_write = twi_callback;
}

void register_twi_read_bmp180(uint8_t (*twi_callback)(uint8_t ack)){
	//rejestracja funkcji odczytuj�cej z i2c
	bmp180_twi_read = twi_callback;
}
