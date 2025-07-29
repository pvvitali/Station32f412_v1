#include "Lcd_1602.h"


uint8_t lcd_led = 0;

void lcd_led_on(){
	lcd_led = 1;
}

void lcd_led_off(){
	lcd_led = 0;
}


HAL_StatusTypeDef lcd_1602_write_8bits_comand(I2C_HandleTypeDef *i2c_handler){
	HAL_StatusTypeDef status;
	uint8_t data[2];
	data[0] = 0x3C;
	data[1] = 0x38;
	status = HAL_I2C_Master_Transmit(i2c_handler, Lcd_1602_address, data, 2, 500);
	return status;
}

HAL_StatusTypeDef lcd_1602_write_4bits_comand(I2C_HandleTypeDef *i2c_handler){
	HAL_StatusTypeDef status;
	uint8_t data[2];
	data[0] = 0x2C;
	data[1] = 0x28;
	status = HAL_I2C_Master_Transmit(i2c_handler, Lcd_1602_address, data, 2, 500);
	return status;
}

HAL_StatusTypeDef lcd_1602_write_comand(I2C_HandleTypeDef *i2c_handler, uint8_t command){
	HAL_StatusTypeDef status;
	uint8_t data[4];
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[0] |= (command & 0xF0) | lcd_led<<3 | 1<<2 ;
	data[1] |= (command & 0xF0) | lcd_led<<3 | 0<<2 ;
	data[2] |= (command & 0x0F)<<4 | lcd_led<<3 | 1<<2 ;
	data[3] |= (command & 0x0F)<<4 | lcd_led<<3 | 0<<2 ;
	status = HAL_I2C_Master_Transmit(i2c_handler, Lcd_1602_address, data, 4, 500);
	return status;
}


HAL_StatusTypeDef Init_lcd_1602(I2C_HandleTypeDef *i2c_handler){
	HAL_StatusTypeDef status;

	HAL_Delay(20);	//del 20ms
	status = lcd_1602_write_8bits_comand(i2c_handler);
	HAL_Delay(5);	//del 5ms
	status = lcd_1602_write_8bits_comand(i2c_handler);
	HAL_Delay(1);	//del 1ms
	status = lcd_1602_write_8bits_comand(i2c_handler);

	HAL_Delay(1);	//del 1ms
	status = lcd_1602_write_4bits_comand(i2c_handler);
	HAL_Delay(1);	//del 1ms
	status = lcd_1602_write_comand(i2c_handler, 0x2C);
	HAL_Delay(1);	//del 1ms
	status = lcd_1602_write_comand(i2c_handler, 0x0C);
	HAL_Delay(1);	//del 1ms
	status = lcd_1602_write_comand(i2c_handler, 0x01);
	HAL_Delay(2);	//del 1ms
	status = lcd_1602_write_comand(i2c_handler, 0x02);
	HAL_Delay(2);	//del 1ms
	status = lcd_1602_write_comand(i2c_handler, 0x06);
	HAL_Delay(1);	//del 1ms

	return status;
}

HAL_StatusTypeDef Lcd_1602_Write_Data(I2C_HandleTypeDef *i2c_handler, uint8_t *pData){
	HAL_StatusTypeDef status;
	uint8_t data[4];
	uint8_t char_data=0;
	uint8_t count=0;
	while(*pData != '\0'){
		char_data = *pData;
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[0] |= (char_data & 0xF0) | lcd_led<<3 | 1<<2 | 1<<0 ;
		data[1] |= (char_data & 0xF0) | lcd_led<<3 | 0<<2 | 1<<0 ;
		data[2] |= (char_data & 0x0F)<<4 | lcd_led<<3 | 1<<2 | 1<<0 ;
		data[3] |= (char_data & 0x0F)<<4 | lcd_led<<3 | 0<<2 | 1<<0 ;
		status = HAL_I2C_Master_Transmit(i2c_handler, Lcd_1602_address, data, 4, 500);
		pData++;

		count++;
		if(count > 20) break;
	}
	return status;
}

HAL_StatusTypeDef Lcd_1602_SetPos(I2C_HandleTypeDef *i2c_handler, uint8_t x, uint8_t y)	//x=offset y=column
{
	HAL_StatusTypeDef status;
	switch(y)
	{
		case 0:
				status = lcd_1602_write_comand(i2c_handler, x|0x80);
				break;
		case 1:
				status = lcd_1602_write_comand(i2c_handler, (0x40+x)|0x80);
				break;
		case 2:
				status = lcd_1602_write_comand(i2c_handler, (0x14+x)|0x80);
				break;
		case 3:
				status = lcd_1602_write_comand(i2c_handler, (0x54+x)|0x80);
				break;
	}
	return status;
}
