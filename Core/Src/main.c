/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
//#include "DS1307.h"
#include "Lcd_1602.h"
#include "Ina_228.h"
#include "FlashPROM.h"
#include "Dwin_lcd.h"
#include "my_atc.h"
#include "station.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

char mas_char[21];
uint16_t adc_potenciometr = 0;
float adc_potect_v = 0;

// variable for function Dwin lcd
// witch called in interrupt
uint16_t pwm_value = 0;
uint16_t on_off_accum_value = 0;

uint16_t u_accum = 0;
float u_accum_float = 0;
_Bool u_220_off = 0;
uint16_t u_12v = 0;

int csq_value = 0;


// energy
float energy_fl = 0;

//hours work
uint32_t hours_work = 0;

_Bool flagDwinProgram = 0;

_Bool flagLcd1602_Enable = 0;

_Bool forse_write_eeprom = 0;

_Bool event_happened = 0;

// time delay
uint32_t time_1 = 0;
uint32_t time_2 = 0;
uint32_t time_3 = 0;
uint32_t time_4 = 0;

//---------------------------
// Sim800 variable
struct telemetriaData telData;
char telString[130] = {0,};
char tempString[15] = {0,};
uint8_t sim_status = 0;
uint8_t flag_sim800_error = 0;
//
char simReadBuff[50] = {0,};
uint8_t countSimReadBuff = 0;
//interrupt sim
volatile uint8_t receiveUart1Byte = 0;
volatile uint8_t flag_is_receive = 0;
uint16_t sim800_on_off = 0;

// I2C n2
HAL_StatusTypeDef status_i2c_2;

//flash PROM
uint32_t res_addr = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t reset_sim800_connect_to_server(void);
void read_sensor_write_lcd(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  if(flagLcd1602_Enable){
	  Init_lcd_1602(&hi2c1);
	  lcd_led_on();
  }

  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_Delay(1500);

	config = 0xFB6A; //config reg address = 1h
	status_i2c_2 = WriteIna228(&hi2c2, 1);
	if( status_i2c_2 != HAL_OK){
	  sprintf(mas_char,"Ina228-conf-error");
	  Dwin_write_str(USART6, 0x5100, (uint8_t *)mas_char, 20, flagDwinProgram);
	  //write status ina228 to lcd
	  if(flagLcd1602_Enable){
		  sprintf(mas_char,"i2-er");
		  Lcd_1602_SetPos(&hi2c1, 0, 0);
		  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
	  }
	}else{
	  sprintf(mas_char,"Ina228-conf-OK");
	  Dwin_write_str(USART6, 0x5100, (uint8_t *)mas_char, 20, flagDwinProgram);
	}
	//
	//config = 500; // current calc reg address = 2h; Rsh=0,002 om
	config = 200; // current calc reg address = 2h; Rsh=0,00045 om 563
	status_i2c_2 = WriteIna228(&hi2c2, 2);
	if( status_i2c_2 != HAL_OK){
	  sprintf(mas_char,"Ina228-conf-error");
	  Dwin_write_str(USART6, 0x5100, (uint8_t *)mas_char, 20, flagDwinProgram);
	  //write status ina228 to lcd
	  if(flagLcd1602_Enable){
		  sprintf(mas_char,"i2-er");
		  Lcd_1602_SetPos(&hi2c1, 0, 0);
		  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
	  }
	}else{
	  sprintf(mas_char,"Ina228-conf-OK");
	  Dwin_write_str(USART6, 0x5100, (uint8_t *)mas_char, 20, flagDwinProgram);
	}
	//
	config = 0;


	// work with flash
	//erase_flash();
	res_addr = flash_search_adress(STARTADDR, BUFFSIZE * DATAWIDTH);
	if( res_addr != STARTADDR){
	  myBuf_t rdata[BUFFSIZE] = {0,}; // буфер для чтения
	  read_last_data_in_flash(rdata); // чтение данных из флеша
	  energy_fl =  *((float*)&rdata[0]);
	  hours_work = rdata[1];
	  pwm_value = (uint16_t)rdata[2];
	  on_off_accum_value = ( (uint16_t)(rdata[2] >> 16) & 0x01 );
	  sim800_on_off = ( (uint16_t)(rdata[2] >> 17) & 0x01 );
	}

	// print kwt
	Dwin_write_float(USART6, 0x5006, energy_fl, flagDwinProgram);
	if(flagLcd1602_Enable){
	  sprintf(mas_char,"%.1fkWh", energy_fl);
	  Lcd_1602_SetPos(&hi2c1, 8, 1);
	  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
	}

	//print hours work
	Dwin_write_int32(USART6, 0x5008, hours_work, flagDwinProgram);

	//print pwm
	Dwin_write_int16(USART6, 0x5012, pwm_value, flagDwinProgram);

	//print status accum
	if(on_off_accum_value == 0){
	  Dwin_write_int16(USART6, 0x5017, 0, flagDwinProgram);
	  Dwin_write_int16(USART6, 0x5022, 0, flagDwinProgram);
	  HAL_GPIO_WritePin(OFF_ACCUM_GPIO_Port, OFF_ACCUM_Pin, GPIO_PIN_RESET);
	}
	if(on_off_accum_value == 1){
	  Dwin_write_int16(USART6, 0x5017, 1, flagDwinProgram);
	  Dwin_write_int16(USART6, 0x5022, 1, flagDwinProgram);
	  HAL_GPIO_WritePin(OFF_ACCUM_GPIO_Port, OFF_ACCUM_Pin, GPIO_PIN_SET);
	}

	// print sim800_on_off
	Dwin_write_int16(USART6, 0x5030, sim800_on_off, flagDwinProgram);


	LL_USART_EnableIT_RXNE(USART3);
	LL_USART_EnableIT_RXNE(USART6);
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_RXNE(USART2);


	sim_status = reset_sim800_connect_to_server();
	//if connect ok == 4
	if( sim_status == 4 ){
		flag_sim800_error = 0;
		Dwin_write_int16(USART6, 0x5020, 1, flagDwinProgram);
		Dwin_write_int16(USART6, 0x5021, 1, flagDwinProgram);
	}
	else{
		flag_sim800_error = 1;
		Dwin_write_int16(USART6, 0x5020, 0, flagDwinProgram);
		Dwin_write_int16(USART6, 0x5021, 0, flagDwinProgram);
		// write to lcd csq
		Dwin_write_int16(USART6, 0x5026, 0, flagDwinProgram);
	}


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  // del 500ms
      if((HAL_GetTick() - time_1) > 500){
    	  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    	  time_1 = HAL_GetTick();
      }




	  // del 300ms
      if((HAL_GetTick() - time_2) > 300){

    	  read_sensor_write_lcd();

    	  time_2 = HAL_GetTick();
      }





      //---------------------------------------------------------
		if( ((HAL_GetTick() - time_4) > 60000) || (event_happened == 1)) { //  1s

		  event_happened = 0;


		if( flag_sim800_error == 0 && sim800_on_off == 1){


			  	defaultStrToStruct(&telData);

			  	sprintf(tempString, "%i", (uint16_t)((current_fl + 0.05) * 10) );
			  	strcpy( telData.i, tempString );

			  	sprintf(tempString, "%i", (uint16_t)((voltage_fl + 0.05) * 10) );
			  	strcpy( telData.u, tempString );

			  	sprintf(tempString, "%i", (uint16_t)((adc_potect_v + 0.005) * 100) );
			  	strcpy( telData.p, tempString );

			  	sprintf(tempString, "%i", (uint16_t)((energy_fl + 0.05) * 10) );
			  	strcpy( telData.power, tempString );

			  	sprintf(tempString, "%i", (uint16_t)(temperature_fl + 0.5) );
			  	strcpy( telData.temperature, tempString );

				// signal quality
				countSimReadBuff = 0;
				sim_status = Sim_Send_Receive(USART1, "AT+CSQ\r\n", &flag_is_receive, &receiveUart1Byte, 0, 100);
				HAL_Delay(10);
				if (sim_status == 1){
						//
						if (simReadBuff[5] == 'Q'){
							if(simReadBuff[9] == ','){
								simReadBuff[9] = '\0';
								strcpy( telData.sq, (char *)&simReadBuff[8]);
							}else{
								simReadBuff[10] = '\0';
								strcpy( telData.sq, (char *)&simReadBuff[8]);
							}
							csq_value = atoi(telData.sq);
						}
				}
				// write to lcd csq
				Dwin_write_int16(USART6, 0x5026, (uint16_t)csq_value, flagDwinProgram);

				//
				sprintf(tempString, "%i", (uint16_t)((u_accum_float + 0.05) * 10) );
				strcpy( telData.u_bat, tempString);
				//
				if( u_12v > 1000){
					sprintf(tempString, "%s", "1");
				}
				else{
					sprintf(tempString, "%s", "0");
				}
				strcpy( telData.u220, tempString);
				//
//				sprintf(tempString, "%+i", u_temperature);
//				strcpy( telData.temperature, tempString);


				// door
				if ( HAL_GPIO_ReadPin(DOOR2_GPIO_Port, DOOR2_Pin) == GPIO_PIN_SET ) {
					//close
					sprintf(tempString, "%s", "0");
				}else {
					//open
					sprintf(tempString, "%s", "1");
				}
				strcpy( telData.door, tempString);


				sprintf(telString, "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|",
						telData.id,
						telData.i,
						telData.u,
						telData.p,
						telData.u_bat,
						telData.sq,
						telData.u220,
						telData.door,
						telData.temperature,
						telData.power,
						telData.i_max,
						telData.t_max
						);
				sprintf(tempString, "%u;\x1A", (unsigned int)crc_8(telString));
				strcat(telString, tempString);


				//resp = ATC_SendReceive(&gsm, "AT+CIPSEND\r\n", 100, NULL, 100, 2, "\r\n> ", "\r\nERROR\r\n");
				sim_status = Sim_Send_Receive(USART1, "AT+CIPSEND\r\n", &flag_is_receive,	&receiveUart1Byte, 0, 1000);
				HAL_Delay(10);

				if (sim_status != 7) {
						flag_sim800_error = 1;
				}else{
						//resp2 = ATC_SendReceive(&gsm, telString, 100, NULL, 2000, 2, "\r\nSEND OK\r\n", "\r\nERROR\r\n");
						sim_status = Sim_Send_Receive(USART1, telString, &flag_is_receive,	&receiveUart1Byte, 0, 2000);
						HAL_Delay(10);
				}

				if (sim_status != 6) flag_sim800_error = 1;

				//sprintf(tempString, "send status -->%u\n", (unsigned int)sim_status);
				//UART3_Transmit( tempString );
		}else{
				//flag_sim800_error = 0;
				sim_status = reset_sim800_connect_to_server();
				//if connect ok == 4
				if( sim_status == 4 ){
					flag_sim800_error = 0;
					Dwin_write_int16(USART6, 0x5020, 1, flagDwinProgram);
					Dwin_write_int16(USART6, 0x5021, 1, flagDwinProgram);
				}
				else{
					flag_sim800_error = 1;
					Dwin_write_int16(USART6, 0x5020, 0, flagDwinProgram);
					Dwin_write_int16(USART6, 0x5021, 0, flagDwinProgram);
					// write to lcd csq
					Dwin_write_int16(USART6, 0x5026, 0, flagDwinProgram);
				}
		}


		time_4 = HAL_GetTick();
      }




      // интервал  1ч
      if( ((HAL_GetTick() - time_3) > 3600000) || (forse_write_eeprom == 1) ){

    	  forse_write_eeprom = 0;

    	  energy_fl = energy_fl + (voltage_fl * current_fl) / 1000;
    	  hours_work = hours_work + 1;
    	  myBuf_t wdata[BUFFSIZE] = {*((uint32_t*)&energy_fl),
    			  	hours_work,
					(uint32_t)pwm_value | ( (uint32_t)on_off_accum_value << 16 ) | ( (uint32_t)sim800_on_off << 17 ),
					0x00000000};
          write_to_flash(wdata); // запись данных во флеш

          // print kwt
          Dwin_write_float(USART6, 0x5006, energy_fl, flagDwinProgram);
          //print hours work
          Dwin_write_int32(USART6, 0x5008, hours_work, flagDwinProgram);
          //
          if(flagLcd1602_Enable){
			  sprintf(mas_char,"%.1fkWh", energy_fl);
			  Lcd_1602_SetPos(&hi2c1, 8, 1);
			  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
          }

          time_3 = HAL_GetTick();
      }





  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 10000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration
  PC5   ------> USART3_RX
  PC10   ------> USART3_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  // GPIO_InitStruct.Pull = LL_GPIO_PULL_UP; !!!
  /**USART3 GPIO Configuration
  PC5   ------> USART3_RX
  PC10   ------> USART3_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**USART6 GPIO Configuration
  PC6   ------> USART6_TX
  PC7   ------> USART6_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART6 interrupt Init */
  NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_Init 1 */

  //LL_GPIO_PULL_UP !!!
  /**USART6 GPIO Configuration
  PC6   ------> USART6_TX
  PC7   ------> USART6_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END USART6_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART6, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART6);
  LL_USART_Enable(USART6);
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CHARGE_ACCUM_Pin|OFF_ACCUM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GATE_V_SIM_Pin|GATE_PWRKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CHARGE_ACCUM_Pin OFF_ACCUM_Pin */
  GPIO_InitStruct.Pin = CHARGE_ACCUM_Pin|OFF_ACCUM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D00R_Pin DOOR2_Pin */
  GPIO_InitStruct.Pin = D00R_Pin|DOOR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GATE_V_SIM_Pin */
  GPIO_InitStruct.Pin = GATE_V_SIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GATE_V_SIM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GATE_PWRKEY_Pin */
  GPIO_InitStruct.Pin = GATE_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GATE_PWRKEY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//==========================================================

uint8_t reset_sim800_connect_to_server() {
		int i;
		uint8_t sim_status;

		if(sim800_on_off == 0){
			HAL_GPIO_WritePin(GATE_PWRKEY_GPIO_Port, GATE_PWRKEY_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GATE_V_SIM_GPIO_Port, GATE_V_SIM_Pin, GPIO_PIN_RESET);
			return 3;	// ret 3 -> error
		}

		//HAL_IWDG_Refresh(&hiwdg); //reset wdg

		HAL_GPIO_WritePin(GATE_PWRKEY_GPIO_Port, GATE_PWRKEY_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GATE_V_SIM_GPIO_Port, GATE_V_SIM_Pin, GPIO_PIN_RESET);
		// delay = 5000ms
		for(int i=0; i < 17; i++){
			read_sensor_write_lcd();	//32ms
			HAL_Delay(270);				//270ms
		}
		HAL_GPIO_WritePin(GATE_V_SIM_GPIO_Port, GATE_V_SIM_Pin, GPIO_PIN_SET);
		// delay = 5000ms
		for(int i=0; i < 17; i++){
			read_sensor_write_lcd();	//32ms
			HAL_Delay(270);				//270ms
		}
		HAL_GPIO_WritePin(GATE_PWRKEY_GPIO_Port, GATE_PWRKEY_Pin, GPIO_PIN_SET);
		// delay = 2000ms
		for(int i=0; i < 7; i++){
			read_sensor_write_lcd();	//32ms
			HAL_Delay(270);				//270ms
		}
		HAL_GPIO_WritePin(GATE_PWRKEY_GPIO_Port, GATE_PWRKEY_Pin, GPIO_PIN_RESET);
		// delay = 500ms
		for(int i=0; i < 2; i++){
			read_sensor_write_lcd();	//32ms
			HAL_Delay(270);				//270ms
		}

    //HAL_IWDG_Refresh(&hiwdg); //reset wdg

    for( i = 0; i < 100; i++) {
    	//HAL_Delay(100);
		read_sensor_write_lcd();	//32ms
		HAL_Delay(270);
    //	HAL_IWDG_Refresh(&hiwdg); //reset wdg
		sim_status = Sim_Send_Receive( USART1,	"AT\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    	if( sim_status == 1 ) break;
    }

    //HAL_IWDG_Refresh(&hiwdg); //reset wdg

    //HAL_Delay(25000);
	// delay = 25000ms
	for(int i=0; i < 84; i++){
		read_sensor_write_lcd();	//32ms
		HAL_Delay(270);				//270ms
	}

    //HAL_IWDG_Refresh(&hiwdg); //reset wdg

    sim_status = Sim_Send_Receive( USART1,	"AT\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // echo off
    sim_status = Sim_Send_Receive( USART1,	"ATE0\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // off all calls
    sim_status = Sim_Send_Receive( USART1,	"AT+GSMBUSY=1\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // sms not in TE
    sim_status = Sim_Send_Receive( USART1,	"AT+CNMI=0,0\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
    if (sim_status != 1) return sim_status;
    // sms text mode
    sim_status = Sim_Send_Receive( USART1,	"AT+CMGF=1\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // signal quality
    sim_status = Sim_Send_Receive( USART1,	"AT+CSQ\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // phone activity status
    sim_status = Sim_Send_Receive( USART1,	"AT+CPAS\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;


    // deactivate gprs context
    sim_status = Sim_Send_Receive( USART1,	"AT+CIPSHUT\r\n", &flag_is_receive,	&receiveUart1Byte,0,5000);
		HAL_Delay(10);
    if (sim_status != 5) return sim_status;
    // single ip conection
    sim_status = Sim_Send_Receive( USART1,	"AT+CIPMUX=0\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // attach gprs
    sim_status = Sim_Send_Receive( USART1,	"AT+CGATT=1\r\n", &flag_is_receive,	&receiveUart1Byte,0,100);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // apn
    sim_status = Sim_Send_Receive( USART1,	"AT+CSTT=\"www\",\"\",\"\"\r\n", &flag_is_receive,	&receiveUart1Byte,0,1000);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // bring wireless connection
    sim_status = Sim_Send_Receive( USART1,	"AT+CIICR\r\n", &flag_is_receive,	&receiveUart1Byte,0,5000);
		HAL_Delay(10);
    if (sim_status != 1) return sim_status;
    // get local ip address => sim_status==0 no OK
    sim_status = Sim_Send_Receive( USART1,	"AT+CIFSR\r\n", &flag_is_receive,	&receiveUart1Byte,0,1000);
		HAL_Delay(10);
    ////if (sim_status != 1) return sim_status; No OK

    sim_status = Sim_Send_Receive( USART1,	"AT+CIPSTART=\"TCP\",\"scz.pge.md\",\"16992\"\r\n", &flag_is_receive,	&receiveUart1Byte,1,5000);
		HAL_Delay(10);
    //HAL_IWDG_Refresh(&hiwdg); //reset wdg

    return sim_status;	//=4 if connect ok
}



// t = 32ms
void read_sensor_write_lcd(void){

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	// read doors
	if( HAL_GPIO_ReadPin(DOOR2_GPIO_Port, DOOR2_Pin) == GPIO_PIN_SET ){
		  // sensor closed
		  //write door
		  Dwin_write_int16(USART6, 0x5016, 1, flagDwinProgram);
		  if(flagLcd1602_Enable){
			  Lcd_1602_SetPos(&hi2c1, 15, 1);
			  Lcd_1602_Write_Data(&hi2c1, (unsigned char *)"1");
		  }
	}else{
		  // sensor open
		  //write door
		  Dwin_write_int16(USART6, 0x5016, 0, flagDwinProgram);
		  if(flagLcd1602_Enable){
			  Lcd_1602_SetPos(&hi2c1, 15, 1);
			  Lcd_1602_Write_Data(&hi2c1, (unsigned char *)"0");
		  }
	}
	//---------------------
	if( HAL_GPIO_ReadPin(D00R_GPIO_Port, D00R_Pin) == GPIO_PIN_SET ){
		  // sensor closed
		  flagDwinProgram = 1;
	}else{
		  // sensor open
		  flagDwinProgram = 0;
	}



	adc_potenciometr = 0;
	HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
	HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	adc_potenciometr = HAL_ADC_GetValue(&hadc1) / 4; // читаем полученное значение в переменную adc
	if( adc_potenciometr > 1001 ){
		  adc_potenciometr = 1001;
	}
	//
	adc_potect_v = 0;
	HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
	HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	adc_potect_v = HAL_ADC_GetValue(&hadc1) * 3.63 / 4095; // читаем полученное значение в переменную adc
	//
	//
	//
	u_accum = 0;
	HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
	HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	u_accum = HAL_ADC_GetValue(&hadc1);
	u_accum_float = u_accum * 3.3 * 2 / 4095; // читаем полученное значение в переменную
	//
	//
	u_12v = 0;
	HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
	HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	u_12v = HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную
	//


	// print potenciometr
	Dwin_write_float(USART6, 0x5010, (float)adc_potenciometr/10.0, flagDwinProgram);
	//
	if(flagLcd1602_Enable){
		  sprintf(mas_char,"%04d", adc_potenciometr);
		  Lcd_1602_SetPos(&hi2c1, 12, 0);
		  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
	}

	//
	if(flagLcd1602_Enable){
		  sprintf(mas_char,"%.2fV", adc_potect_v);
		  Lcd_1602_SetPos(&hi2c1, 6, 0);
		  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
	}

	//print on off accum
	//set on 0ff accum
	if(on_off_accum_value == 0){
		  Dwin_write_int16(USART6, 0x5022, 0, flagDwinProgram);
		  HAL_GPIO_WritePin(OFF_ACCUM_GPIO_Port, OFF_ACCUM_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(CHARGE_ACCUM_GPIO_Port, CHARGE_ACCUM_Pin, GPIO_PIN_RESET);
		  // print v accum
		  Dwin_write_float(USART6, 0x5024, 0.0, flagDwinProgram);
	}
	if(on_off_accum_value == 1){
		  Dwin_write_int16(USART6, 0x5022, 1, flagDwinProgram);
		  HAL_GPIO_WritePin(OFF_ACCUM_GPIO_Port, OFF_ACCUM_Pin, GPIO_PIN_SET);
		  // print v accum
		  Dwin_write_float(USART6, 0x5024, u_accum_float, flagDwinProgram);

		  // set on off charge accum
		// u_accum = 2482; v = 4.0v
		// u_accum = 2234; v = 3.5v
		  if( u_accum > 2482 ){	// u_accum = 2482; v = 4.0v
			  HAL_GPIO_WritePin(CHARGE_ACCUM_GPIO_Port, CHARGE_ACCUM_Pin, GPIO_PIN_RESET);
		  }
		  if( u_accum < 2382 ){
			  HAL_GPIO_WritePin(CHARGE_ACCUM_GPIO_Port, CHARGE_ACCUM_Pin, GPIO_PIN_SET);
		  }
		  // Off power
		  if( (u_accum < 2234) && (u_220_off == 1) ){	// u_accum = 2234; v = 3.5v and off 220v => off accum
			  HAL_GPIO_WritePin(OFF_ACCUM_GPIO_Port, OFF_ACCUM_Pin, GPIO_PIN_RESET);
		  }
	}


	//print pwm
	Dwin_write_int16(USART6, 0x5012, pwm_value, flagDwinProgram);
	// set PWM
	TIM2->CCR1 = pwm_value;
	TIM2->CCR2 = 500;


	// read ina228
	status_i2c_2 = ReadIna228(&hi2c2);
	if( status_i2c_2 != HAL_OK){
		  sprintf(mas_char,"i2c-ReadIna228-error");
		  Dwin_write_str(USART6, 0x5100, (uint8_t *)mas_char, 20, flagDwinProgram);
		  if(flagLcd1602_Enable){
			  sprintf(mas_char,"i2-er");
			  Lcd_1602_SetPos(&hi2c1, 0, 0);
			  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
		  }

	}else{
		  sprintf(mas_char,"i2c-ReadIna228-OK");
		  Dwin_write_str(USART6, 0x5100, (uint8_t *)mas_char, 20, flagDwinProgram);
		  //
		  if(flagLcd1602_Enable){
			  if( sprintf(mas_char,"%.1fV", voltage_fl) == 4){
				  sprintf(mas_char," %.1fV", voltage_fl);
			  }else{
				  sprintf(mas_char,"%.1fV", voltage_fl);
			  }
			  Lcd_1602_SetPos(&hi2c1, 0, 0);
			  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
		  }
		  //
		  // print voltage
		  Dwin_write_float(USART6, 0x5000, voltage_fl, flagDwinProgram);
		  // print current
		  Dwin_write_float(USART6, 0x5002, current_fl, flagDwinProgram);;
		  // print current
		  Dwin_write_float(USART6, 0x5028, temperature_fl, flagDwinProgram);
		  //
		  if(flagLcd1602_Enable){
			  if( sprintf(mas_char,"%.1fV", current_fl) == 4){
				  sprintf(mas_char," %.1fA ", current_fl);
			  }else{
				  sprintf(mas_char,"%.1fA ", current_fl);
			  }
			  Lcd_1602_SetPos(&hi2c1, 0, 1);
			  Lcd_1602_Write_Data(&hi2c1, (uint8_t *)mas_char);
		  }

	}

	// print potencial
	Dwin_write_float(USART6, 0x5004, adc_potect_v, flagDwinProgram);

//	if(sim800_on_off == 0){
//		HAL_GPIO_WritePin(GATE_PWRKEY_GPIO_Port, GATE_PWRKEY_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GATE_V_SIM_GPIO_Port, GATE_V_SIM_Pin, GPIO_PIN_RESET);
//		Dwin_write_int16(USART6, 0x5020, 0, flagDwinProgram);
//		Dwin_write_int16(USART6, 0x5021, 0, flagDwinProgram);
//	}else{
//		if(flag_sim800_error == 1){
//			Dwin_write_int16(USART6, 0x5020, 0, flagDwinProgram);
//			Dwin_write_int16(USART6, 0x5021, 0, flagDwinProgram);
//		}else{
//			Dwin_write_int16(USART6, 0x5020, 1, flagDwinProgram);
//			Dwin_write_int16(USART6, 0x5021, 1, flagDwinProgram);
//		}
//	}

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
