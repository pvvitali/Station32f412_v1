/*
 * Dwin_lcd.c
 *
 *  Created on: Jul 20, 2025
 *      Author: vital
 */
#include "Dwin_lcd.h"

#define SIZE_BUFF_DWIN 25
uint8_t tx_buff_toDisplay[SIZE_BUFF_DWIN] = {0,};

void Dwin_write_float(USART_TypeDef *USARTx, uint16_t address, float value_fl, _Bool flagDwinProgram){

	  if(flagDwinProgram) return;

	  uint32_t value_int = *((uint32_t *)&value_fl);
	  //
	  tx_buff_toDisplay[0] = 0x5A;
	  tx_buff_toDisplay[1] = 0xA5;
	  tx_buff_toDisplay[2] = 0x07;
	  tx_buff_toDisplay[3] = 0x82;
	  tx_buff_toDisplay[4] = (uint8_t)(address >> 8);
	  tx_buff_toDisplay[5] = (uint8_t)address;
	  tx_buff_toDisplay[6] = (uint8_t)(value_int >> 24);
	  tx_buff_toDisplay[7] = (uint8_t)(value_int >> 16);
	  tx_buff_toDisplay[8] = (uint8_t)(value_int >> 8);
	  tx_buff_toDisplay[9] = (uint8_t)value_int;
	  //
	  for(int i = 0; i < 10; i++){
			while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
			LL_USART_TransmitData8(USARTx, tx_buff_toDisplay[i]);
	  }
}


void Dwin_write_int16(USART_TypeDef *USARTx, uint16_t address, uint16_t value, _Bool flagDwinProgram){

	  if(flagDwinProgram) return;

	  tx_buff_toDisplay[0] = 0x5A;
	  tx_buff_toDisplay[1] = 0xA5;
	  tx_buff_toDisplay[2] = 0x05;
	  tx_buff_toDisplay[3] = 0x82;
	  tx_buff_toDisplay[4] = (uint8_t)(address >> 8);
	  tx_buff_toDisplay[5] = (uint8_t)address;
	  tx_buff_toDisplay[6] = (uint8_t)(value >> 8);
	  tx_buff_toDisplay[7] = (uint8_t)value;
	  //
	  for(int i = 0; i < 8; i++){
			while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
			LL_USART_TransmitData8(USARTx, tx_buff_toDisplay[i]);
	  }
}

void Dwin_write_int32(USART_TypeDef *USARTx, uint16_t address, uint32_t value, _Bool flagDwinProgram){

	  if(flagDwinProgram) return;

	  tx_buff_toDisplay[0] = 0x5A;
	  tx_buff_toDisplay[1] = 0xA5;
	  tx_buff_toDisplay[2] = 0x07;
	  tx_buff_toDisplay[3] = 0x82;
	  tx_buff_toDisplay[4] = (uint8_t)(address >> 8);
	  tx_buff_toDisplay[5] = (uint8_t)address;
	  tx_buff_toDisplay[6] = (uint8_t)(value >> 24);
	  tx_buff_toDisplay[7] = (uint8_t)(value >> 16);
	  tx_buff_toDisplay[8] = (uint8_t)(value >> 8);
	  tx_buff_toDisplay[9] = (uint8_t)value;
	  //
	  for(int i = 0; i < 10; i++){
			while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
			LL_USART_TransmitData8(USARTx, tx_buff_toDisplay[i]);
	  }
}


void Dwin_write_str(USART_TypeDef *USARTx, uint16_t address, uint8_t * buff, uint8_t dwin_text_length, _Bool flagDwinProgram){

	if(flagDwinProgram) return;

	uint8_t size = 0;
	for(int i = 0; i < SIZE_BUFF_DWIN; i++){
		if( buff[i] == '\0' ) size = i;
	}
	if(size == 0) return;

	// write new string
	tx_buff_toDisplay[0] = 0x5A;
	tx_buff_toDisplay[1] = 0xA5;
	tx_buff_toDisplay[2] = size + 3;
	tx_buff_toDisplay[3] = 0x82;
	tx_buff_toDisplay[4] = (uint8_t)(address >> 8);
	tx_buff_toDisplay[5] = (uint8_t)address;
	for(int i = 0; i < 6; i++){
		while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
		LL_USART_TransmitData8(USARTx, tx_buff_toDisplay[i]);
	}
	for(int i = 0; i < size; i++){
		while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
		LL_USART_TransmitData8(USARTx, buff[i]);
	}
	for(int i = size; i < dwin_text_length; i++){
		while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
		LL_USART_TransmitData8(USARTx, 0x20);	// write space
	}
}

//return 0 - else not fined; 1 - data fined and receiveUartByted
uint8_t Dwin_receive_data(uint8_t receiveUartByte,
							uint16_t address,
							uint16_t * data_receive
							){
	static uint8_t faze_sim = 0;
	static uint8_t data_receive_high = 0;
	static uint8_t data_receive_low = 0;

	// find char =============================================
	switch (faze_sim){
		case 0:
			if(receiveUartByte == 0x5A) faze_sim = 1;
			break;

		case 1:
			if(receiveUartByte == 0x5A){
				faze_sim = 1;
				break;
			}
			if(receiveUartByte == 0xA5){
				faze_sim = 2;
				break;
			}
			faze_sim = 0;
			break;

		case 2:
			if(receiveUartByte == 0x06){	//size data of dwin packet
				faze_sim = 3;
				break;
			}
			faze_sim = 0;
			break;

		case 3:
			if(receiveUartByte == 0x83 ){
				faze_sim = 4;
				break;
			}
			faze_sim = 0;
			break;

		case 4:
			if(receiveUartByte == (uint8_t)(address >> 8) ){
				faze_sim = 5;
				break;
			}
			faze_sim = 0;
			break;

		case 5:
			if(receiveUartByte == (uint8_t)address ){
				faze_sim = 6;
				break;
			}
			faze_sim = 0;
			break;

		case 6:
			if(receiveUartByte == 0x01){
				faze_sim = 7;
				break;
			}
			faze_sim = 0;
			break;

		case 7:
			data_receive_high = receiveUartByte;
			faze_sim = 8;
			break;

		case 8:
			data_receive_low = receiveUartByte;
			faze_sim = 0;
			*data_receive = ( ((uint16_t)data_receive_high << 8) & 0xFF00) | ((uint16_t)data_receive_low & 0x00FF);
			return 1;
	} //end switch


	return 0;

}


//return 0 - else not fined; 1 - data fined and receiveUartByted
uint8_t Dwin_receive_data_2(uint8_t receiveUartByte,
							uint16_t address,
							uint16_t * data_receive
							){
	static uint8_t faze_sim = 0;
	static uint8_t data_receive_high = 0;
	static uint8_t data_receive_low = 0;

	// find char =============================================
	switch (faze_sim){
		case 0:
			if(receiveUartByte == 0x5A) faze_sim = 1;
			break;

		case 1:
			if(receiveUartByte == 0x5A){
				faze_sim = 1;
				break;
			}
			if(receiveUartByte == 0xA5){
				faze_sim = 2;
				break;
			}
			faze_sim = 0;
			break;

		case 2:
			if(receiveUartByte == 0x06){	//size data of dwin packet
				faze_sim = 3;
				break;
			}
			faze_sim = 0;
			break;

		case 3:
			if(receiveUartByte == 0x83 ){
				faze_sim = 4;
				break;
			}
			faze_sim = 0;
			break;

		case 4:
			if(receiveUartByte == (uint8_t)(address >> 8) ){
				faze_sim = 5;
				break;
			}
			faze_sim = 0;
			break;

		case 5:
			if(receiveUartByte == (uint8_t)address ){
				faze_sim = 6;
				break;
			}
			faze_sim = 0;
			break;

		case 6:
			if(receiveUartByte == 0x01){
				faze_sim = 7;
				break;
			}
			faze_sim = 0;
			break;

		case 7:
			data_receive_high = receiveUartByte;
			faze_sim = 8;
			break;

		case 8:
			data_receive_low = receiveUartByte;
			faze_sim = 0;
			*data_receive = ( ((uint16_t)data_receive_high << 8) & 0xFF00) | ((uint16_t)data_receive_low & 0x00FF);
			return 1;
	} //end switch


	return 0;

}


//return 0 - else not fined; 1 - data fined and receiveUartByted
uint8_t Dwin_receive_data_3(uint8_t receiveUartByte,
							uint16_t address,
							uint16_t * data_receive
							){
	static uint8_t faze_sim = 0;
	static uint8_t data_receive_high = 0;
	static uint8_t data_receive_low = 0;

	// find char =============================================
	switch (faze_sim){
		case 0:
			if(receiveUartByte == 0x5A) faze_sim = 1;
			break;

		case 1:
			if(receiveUartByte == 0x5A){
				faze_sim = 1;
				break;
			}
			if(receiveUartByte == 0xA5){
				faze_sim = 2;
				break;
			}
			faze_sim = 0;
			break;

		case 2:
			if(receiveUartByte == 0x06){	//size data of dwin packet
				faze_sim = 3;
				break;
			}
			faze_sim = 0;
			break;

		case 3:
			if(receiveUartByte == 0x83 ){
				faze_sim = 4;
				break;
			}
			faze_sim = 0;
			break;

		case 4:
			if(receiveUartByte == (uint8_t)(address >> 8) ){
				faze_sim = 5;
				break;
			}
			faze_sim = 0;
			break;

		case 5:
			if(receiveUartByte == (uint8_t)address ){
				faze_sim = 6;
				break;
			}
			faze_sim = 0;
			break;

		case 6:
			if(receiveUartByte == 0x01){
				faze_sim = 7;
				break;
			}
			faze_sim = 0;
			break;

		case 7:
			data_receive_high = receiveUartByte;
			faze_sim = 8;
			break;

		case 8:
			data_receive_low = receiveUartByte;
			faze_sim = 0;
			*data_receive = ( ((uint16_t)data_receive_high << 8) & 0xFF00) | ((uint16_t)data_receive_low & 0x00FF);
			return 1;
	} //end switch


	return 0;

}
