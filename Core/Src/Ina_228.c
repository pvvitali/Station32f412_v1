#include "Ina_228.h"
#include "stdio.h"

uint8_t mass_ina228_data[4];
uint8_t mass_i_data[7];
uint8_t mass_v_data[7];

//vars voltage, current
uint16_t config;

uint32_t voltage_int;
float voltage_fl;

uint32_t current_int;
float current_fl;

uint16_t temperature_int;
float temperature_fl;

uint8_t mass_temp_data[10];

//чтение 16-битных регистров: Configuration, Shunt voltage, Bus voltage_int
//to -> uint8_t mass_ina228_data[2];
HAL_StatusTypeDef ReadIna228(I2C_HandleTypeDef * i2c_handler){
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(i2c_handler, Ina228_Address, 0x1, I2C_MEMADD_SIZE_8BIT, mass_ina228_data, 2, 100);
	if (status != HAL_OK){
		return status;
	}else{
		config = mass_ina228_data[0]<<8 | mass_ina228_data[1];
	}


	// read voltage
	status = HAL_I2C_Mem_Read(i2c_handler, Ina228_Address, 0x5, I2C_MEMADD_SIZE_8BIT, mass_ina228_data, 3, 100);
	if (status != HAL_OK){
		return status;
	}else{
		voltage_int = mass_ina228_data[0]<<16 | mass_ina228_data[1]<<8 | mass_ina228_data[2];
		voltage_int = voltage_int >> 4;
		voltage_fl = voltage_int * 0.0001953125 ;

	}

	// read temperature
	status = HAL_I2C_Mem_Read(i2c_handler, Ina228_Address, 0x6, I2C_MEMADD_SIZE_8BIT, mass_ina228_data, 2, 100);
	if (status != HAL_OK){
		return status;
	}else{
		temperature_int = mass_ina228_data[0]<<8 | mass_ina228_data[1];
		temperature_fl = (temperature_int * 7.8125) / 1000.0 ;
	}



	// read current
	status = HAL_I2C_Mem_Read(i2c_handler, Ina228_Address, 0x7, I2C_MEMADD_SIZE_8BIT, mass_ina228_data, 3, 100);
	if (status != HAL_OK){
		return status;
	}else{
		current_int = mass_ina228_data[0]<<16 | mass_ina228_data[1]<<8 | mass_ina228_data[2];
		current_int = current_int >> 4;
		if(current_int & 0x80000) {
			current_int = 0;
		}
		//current_fl = current_int * 0.00001907 ;
		current_fl = current_int * 0.00009537 ;
	}


	return status;
}


//запись 16битного конфигурационного регистра
//in -> uint8_t mass_ina228_data[6];
HAL_StatusTypeDef WriteIna228(I2C_HandleTypeDef * i2c_handler, uint8_t address_conf_reg){
	HAL_StatusTypeDef status;
	mass_ina228_data[0] = 0x00FF & (config >> 8);
	mass_ina228_data[1] = 0x00FF & config;
	status = HAL_I2C_Mem_Write(i2c_handler, Ina228_Address, address_conf_reg, I2C_MEMADD_SIZE_8BIT, mass_ina228_data, 2, 100);

	return status;
}



