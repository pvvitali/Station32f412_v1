#include "my_atc.h"


void delay_cycles(uint8_t it){
	while(it--);
}

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
						){
	uint8_t faze_sim = 0;
	uint8_t ansver = 0;
	uint8_t receive = 0;
	uint32_t inner_time;
	//
	*receiveUart1Byte = 0;
	*flag_is_receive = 0;
	
	//transfer
	while( *buf != '\0' ){
//			while( !UART1_GetFlagStatus(UART1_FLAG_TXE) );
//			UART1_SendData8((uint8_t)*buf);
			while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
			LL_USART_TransmitData8(USARTx, (uint8_t)*buf);
			buf++;
	}

	
	
	// ==========================================
	inner_time = HAL_GetTick();  // fixed start time!
	
	
	while(1){
		
			// waite receive
			while(*flag_is_receive == 0){
				//reset WatchDog
				//IWDG_ReloadCounter();
				delay_cycles(10); // delay ~100cicles
				//
				if(  (HAL_GetTick() - inner_time) > delay) { //  delay in ms
					return 2;		// 2 - TimeOut
				}
			}
			// receive get on
			receive = *receiveUart1Byte;
			*flag_is_receive = 0;
			
			// find char =============================================
			switch (faze_sim){
				case 0:
					if(receive == '\r') faze_sim = 1;
					break;
					
				case 1:
					if(receive == '\r'){
						faze_sim = 1;
						break;
					}
					if(receive == '\n'){
						faze_sim = 2;
						break;
					}
					faze_sim = 0;
					break;
					
				case 2:
					if(receive == '\r'){
						faze_sim = 1;
						break;
					}
					if(receive == 'O'){
						faze_sim = 3;
						ansver = 1;		// wait OK
						break;
					}
					if(receive == 'E'){
						faze_sim = 3;
						ansver = 3;		// wait ERROR
						break;
					}
					if(receive == 'C'){
						faze_sim = 3;
						ansver = 4;		// wait CONNECT OK
						break;
					}
					if(receive == 'S'){
						faze_sim = 3;
						ansver = 5;		// wait SHUT OK or wait SEND OK
						break;
					}
					if(receive == '>'){
						faze_sim = 3;
						ansver = 7;		// wait >
						break;
					}
					faze_sim = 0;
					break;
					
					case 3:
					if(receive == '\r'){
						faze_sim = 1;
						break;
					}
					if(ansver == 1 && receive == 'K'){	// wait OK	
						faze_sim = 4;
						break;
					}
					if(ansver == 3 && receive == 'R'){	// wait ERROR	
						faze_sim = 4;
						break;
					}
					if(ansver == 4 && receive == 'O'){	// wait CONNECT OK	
						faze_sim = 4;
						break;
					}
					if(ansver == 5 && receive == 'H'){	// wait SHUT OK	
						faze_sim = 4;
						break;
					}
					if(ansver == 5 && receive == 'E'){	// wait SEND OK	
						ansver = 6;
						faze_sim = 4;
						break;
					}
					if(ansver == 7 && receive == ' '){	// find >	
						return 7;
					}
					faze_sim = 0;
					break;
					
					case 4:
					if(ansver == 1 && receive == '\r'){	// find OK	
							if( find_connect_ok == 0 ) {	// no find connect ok
									return 1;		// find OK
							}else{
									faze_sim = 1;
									break;
							}
					}
					if(ansver == 3 && receive == 'R'){	// wait ERROR	
							return 3;		// find ERROR
					}
					if(ansver == 4 && receive == 'N'){	// wait CONNECT OK	
						faze_sim = 5;
						break;
					}
					if(ansver == 5 && receive == 'U'){	// wait SHUT OK	
							return 5;
					}
					if(ansver == 6 && receive == 'N'){	// wait SEND OK		
						faze_sim = 5;
						break;
					}
					if(receive == '\r'){
						faze_sim = 1;
						break;
					}
					faze_sim = 0;
					break;
					
					case 5:
						if(receive == '\r'){
							faze_sim = 1;
							break;
						}
						faze_sim = 6;
						break;
					
					case 6:
						if(receive == '\r'){
							faze_sim = 1;
							break;
						}
						faze_sim = 7;
						break;
					
					case 7:
						if(receive == '\r'){
							faze_sim = 1;
							break;
						}
						if(ansver == 6 && receive == 'O'){	// find SEND OK	
							return 6;
						}
						faze_sim = 8;
						break;
					
					case 8:
						if(receive == '\r'){
							faze_sim = 1;
							break;
						}
						faze_sim = 9;
						break;
					
					case 9:
						if(receive == '\r'){
							faze_sim = 1;
							break;
						}
						faze_sim = 10;
						break;
					
					case 10:
						if(receive == '\r'){
							faze_sim = 1;
							break;
						}
						if(ansver == 4 && receive == 'O'){	// find CONNECT OK	
							return 4;
						}
						faze_sim = 0;
						break;			
			} //end switch
			
	} //end while(1)
	
	return 10;		// never return, for check			
	
} // end Sim_Send_Receive
