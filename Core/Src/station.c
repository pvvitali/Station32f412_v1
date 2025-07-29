#include "station.h"

uint8_t crc_8(char *str){
	uint8_t crc = 0xFF;
	uint8_t e = 0;
	uint8_t i =0;
	for(e = 0; str[e] != '\0'; e++){
		crc = crc ^ str[e];
		for(i =0; i < 8; i++){
			if((crc & 0x01) != 0){
				crc = (crc>>1) ^ 0x8C;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

void defaultStrToStruct(TelData *telData){
	strcpy(telData->id, ID);
	strcpy(telData->i, "0");
	strcpy(telData->u, "0");
	strcpy(telData->p, "0");
	strcpy(telData->u_bat, "0");
	strcpy(telData->sq, "0");
	strcpy(telData->u220, "0");
	strcpy(telData->door, "0");
	strcpy(telData->temperature, "0");
	strcpy(telData->power, "0");
	strcpy(telData->i_max, "0");
	strcpy(telData->t_max, "0");
}

void decodeStrToStruct(TelData *telData, char *str){

	uint8_t j =0;
	uint8_t k = 0;
	uint8_t i = 1;
	char tempStr[10];

	strcpy(telData->id, ID);
	strcpy(telData->i, "0");
	strcpy(telData->u, "0");
	strcpy(telData->p, "0");
	strcpy(telData->u_bat, "0");
	strcpy(telData->sq, "0");
	strcpy(telData->u220, "0");
	strcpy(telData->door, "0");
	strcpy(telData->temperature, "0");
	strcpy(telData->power, "0");
	strcpy(telData->i_max, "0");
	strcpy(telData->t_max, "0");

	if (str[0] != '$') return;
	for(i = 1; (str[i] != '\0') && (i < 50); i++){
		if( str[i] != '|') {
			if( str[i] == ';') break;
			tempStr[k] = str[i];
			k++;
		}
		else {
			tempStr[k] = '\0';
			if( j == 0) if(strlen(tempStr)<9) strcpy( telData->i, tempStr );
			if( j == 1) if(strlen(tempStr)<9) strcpy( telData->u, tempStr );
			if( j == 2) if(strlen(tempStr)<9) strcpy( telData->p, tempStr );
			if( j == 3) if(strlen(tempStr)<9) strcpy( telData->i_max, tempStr );
			if( j == 4) if(strlen(tempStr)<9) strcpy( telData->t_max, tempStr );
			if( j == 5) if(strlen(tempStr)<14) strcpy( telData->power, tempStr );
			k = 0;
			j++;
		}
	}
}

int strServerToStation(char * station, char * server, int lenStation){
	char val[5] = {0,};
	char crc[5] = {0,};
	char str[15] = {0,};
	int value = 0;
	int crc_int = 0;

	int lenServer = strlen(server);
	int position2 = 0;
	int i,k = 0;
	for(i=0; i < lenStation; i++){
		station[i] = '\0';
	}
	if (server[0] != '$') return 1;
	if (server[1] != '2') return 1;
	if (server[2] != '|') return 1;

	for( i=3,k=0; i < lenServer; i++,k++ ){
		if(server[i] != '|') val[k] = server[i];
		else break;
	}
	position2 = i;
	i++;
	for( k=0; i < lenServer; i++,k++ ){
		crc[k] = server[i];
	}
	for(i = 0; i <= position2; i++){
		str[i] = server[i];
	}
	str[i] = '\0';
	//
	value = atoi(val);
	crc_int = atoi(crc);
	//
	if ( crc_int != crc_8(str)) return 1;
	value *= 3;
	sprintf( station, "$R%03d;", value);
	return 0;
}




//реализация crc8
//static byte Crc8(string data)
//        {
//            var len = data.Length;
//            int crc = 0xFF;
//            for (int e = 0; e < len; e++)
//            {
//                crc = crc ^ data[e];
//                for (int i = 0; i < 8; i++)
//                {
//                    if ((crc & 0x01) != 0)
//                        crc = (crc >> 1) ^ 0x8C;
//                    else
//                        crc >>= 1;
//                }
//            }
//            return (byte)crc;
//        }
