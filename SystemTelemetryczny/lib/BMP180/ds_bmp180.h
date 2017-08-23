/*
 * ds_bmp180.h
 *
 *  Created on: 21-12-2015
 *      Author: Daniel
 *		   web: www.danielo88.cba.pl
 *
 *
 *	UWAGA!!!
 *
 *	W tym pliku nie nale¿y niczego zmieniaæ
 *
 */

#ifndef DS_BMP180_H_
#define DS_BMP180_H_

//odpowiednie ustawienie bitów dla oversampling w rejestrze 0xF4
#define BMP180_OSS_1	0
#define BMP180_OSS_2	1
#define BMP180_OSS_4	2
#define BMP180_OSS_8	3

//*****	struktury i zmienne widoczne na zewn¹trz

typedef struct{
	int32_t	bmp180_UT;					//wartosc UT odczytana z uk³adu
	int32_t bmp180_UP;					//wartosc UP odczytana z uk³adu
	uint8_t bmp180_temp_sign;			//temperatura - znak
	uint8_t bmp180_temp_cell;			//temperatura - calosc
	uint8_t bmp180_temp_frac;			//temperatura - wartosc po przecinku
	uint16_t bmp180_pressure_cell;		//cisnienie - calosc
	uint8_t bmp180_pressure_frac;		//cisnienie - wartosc po przecinku
	uint8_t chip_id;					//id chipu - 0x55
	uint8_t ver_regist;					//wersja rejestru
}DS_BMP180;

extern DS_BMP180	ds_bmp180;
extern volatile uint8_t bmp180_time_1ms;

//*****	funkcje widoczne na zewn¹trz

uint8_t ds_bmp180_coeff(void);
uint8_t ds_bmp180_measure(void);
void ds_bmp180_start(uint8_t pres_oversampling);
void ds_bmp180_restart(void);
uint8_t ds_bmp180_active(void);

//*****	rejestracja funkcji i2c potrzebnych do prawid³owego dzia³ania biblioteki
void register_twi_start_bmp180(void (*twi_callback)(void));				//rejestracja funkcji start z i2c
void register_twi_stop_bmp180(void (*twi_callback)(void));				//rejestracja funkcji stop z i2c
void register_twi_write_bmp180(void (*twi_callback)(uint8_t bajt));		//rejestracja funkcji zapisuj¹cej do i2c
void register_twi_read_bmp180(uint8_t (*twi_callback)(uint8_t ack));		//rejestracja funkcji odczytuj¹cej z i2c


#endif /* DS_BMP180_H_ */
