#ifndef INC_LCD_1602_H_
#define INC_LCD_1602_H_

#include "stm32f4xx_hal.h"

#define Lcd_1602_address	0x4E    //0x70



void lcd_led_on();
void lcd_led_off();

HAL_StatusTypeDef lcd_1602_write_8bits_comand(I2C_HandleTypeDef *i2c_handler);
HAL_StatusTypeDef lcd_1602_write_4bits_comand(I2C_HandleTypeDef *i2c_handler);

HAL_StatusTypeDef lcd_1602_write_comand(I2C_HandleTypeDef *i2c_handler, uint8_t command);

HAL_StatusTypeDef Init_lcd_1602(I2C_HandleTypeDef *i2c_handler);

HAL_StatusTypeDef Lcd_1602_Write_Data(I2C_HandleTypeDef *i2c_handler, uint8_t *pData);
HAL_StatusTypeDef Lcd_1602_SetPos(I2C_HandleTypeDef *i2c_handler, uint8_t x, uint8_t y);	//x=offset y=column


#endif /* INC_LCD_1602_H_ */
