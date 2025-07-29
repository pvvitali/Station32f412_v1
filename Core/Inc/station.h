
#ifndef __STATION_H__
#define __STATION_H__


//#include "stm8s.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define ID "10001001"

uint8_t crc_8(char *str);


struct telemetriaData {
	char id[10];
	char i[10];
	char u[10];
	char p[10];
	char u_bat[10];
	char sq[10];
	char u220[10];
	char door[10];
	char temperature[10];
	char power[15];
	char i_max[10];
	char t_max[10];
};

typedef struct telemetriaData TelData;

void defaultStrToStruct(TelData *telData);
void decodeStrToStruct(TelData *telData, char *str);
int strServerToStation(char * station, char * server, int lenStation);

#endif /* __STATION_H__ */

