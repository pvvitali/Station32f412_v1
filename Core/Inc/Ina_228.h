#ifndef INC_INA_228_H_
#define INC_INA_228_H_

#include "stm32f4xx_hal.h"

#define Ina228_Address	0x80	//0b10000000

extern uint8_t mass_ina228_data[4];
extern uint8_t mass_i_data[7];
extern uint8_t mass_v_data[7];

//vars voltage, current
extern uint16_t config;
extern float voltage_fl;
extern float current_fl;
extern float temperature_fl;


//чтение 16битных регистров: Configuration, Shunt voltage, Bus voltage
//to -> uint8_t mass_ina228_data[4];
HAL_StatusTypeDef ReadIna228(I2C_HandleTypeDef * i2c_handler);

//запись 16битного конфигурационного регистра
//in -> uint8_t mass_ina219_data[4];
HAL_StatusTypeDef WriteIna228(I2C_HandleTypeDef * i2c_handler, uint8_t address_conf_reg);

#endif /* INC_INA_228_H_ */
