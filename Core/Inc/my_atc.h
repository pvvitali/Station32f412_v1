#ifndef __MY_ATC_H
#define __MY_ATC_H

//#include "stm8s.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_usart.h"

void delay_cycles(uint8_t it);


// Use UART1 (change for another)
//==================================
// Return value:
// 1 - OK
// 2 - TimeOut
// 3 - Err
// 4 - CONNTCT OK
// 5 - SHUT OK
// 6 - SEND OK	
// 7 - find >
//==================================
// find_connect_ok == 0 - no find connecn ok -> find only ok
// find_connect_ok == 1 - find connecn ok -> �� ignore
uint8_t Sim_Send_Receive(	USART_TypeDef *USARTx,
							char *buf,
							volatile uint8_t *flag_is_receive,
							volatile uint8_t *receiveUart1Byte,
							uint8_t find_connect_ok,
							uint16_t delay
						);

#endif
