/*
 * Dwin_lcd.h
 *
 *  Created on: Jul 20, 2025
 *      Author: vital
 */

#ifndef INC_DWIN_LCD_H_
#define INC_DWIN_LCD_H_

#include "stm32f4xx.h"
#include "stm32f4xx_ll_usart.h"

void Dwin_write_float(USART_TypeDef *USARTx, uint16_t address, float value, _Bool flagDwinProgram);
void Dwin_write_int16(USART_TypeDef *USARTx, uint16_t address, uint16_t value, _Bool flagDwinProgram);
void Dwin_write_int32(USART_TypeDef *USARTx, uint16_t address, uint32_t value, _Bool flagDwinProgram);
void Dwin_write_str(USART_TypeDef *USARTx, uint16_t address, uint8_t * buff, uint8_t dwin_text_length, _Bool flagDwinProgram);
//
//return 0 - else not fined; 1 - data fined and received
uint8_t Dwin_receive_data(uint8_t receiveUartByte, uint16_t address, uint16_t * data_received);
//return 0 - else not fined; 1 - data fined and received
uint8_t Dwin_receive_data_2(uint8_t receiveUartByte, uint16_t address, uint16_t * data_received);//return 0 - else not fined; 1 - data fined and receiveUartByted
uint8_t Dwin_receive_data_3(uint8_t receiveUartByte,
							uint16_t address,
							uint16_t * data_receive
							);

#endif /* INC_DWIN_LCD_H_ */
