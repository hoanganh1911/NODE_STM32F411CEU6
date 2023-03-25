/*
 * keypad_pcf.c
 *
 *  Created on: Mar 4, 2023
 *      Author: hoanganh
 */

#include <Keypad.h>
uint8_t read(I2C_HandleTypeDef *_hi2c,uint8_t mask)
{
	uint8_t data;
	HAL_I2C_Master_Transmit(_hi2c, PCF_ADDR << 1, &mask,1, 1000);
	HAL_I2C_Master_Receive(_hi2c, PCF_ADDR << 1, &data,1, 1000);
	return data;
}
uint8_t getkey(I2C_HandleTypeDef *_hi2c)
{
	uint8_t key = 0;
	uint8_t lastkey = 0;
	uint8_t cols = read(_hi2c,0x0F);

	if (cols == 0x0F)
		return 17;
	else if(cols == 0x07)
		key = 3;
	else if(cols == 0x0B)
		key = 2;
	else if(cols == 0x0D)
		key = 1;
	else if(cols == 0x0E)
		key = 0;

	uint8_t rows = read(_hi2c,0xF0);
	if (rows == 0xF0)
		return 17;
	else if(rows == 0x70)
		key+=12;
	else if (rows == 0xB0)
		key+=8;
	else if (rows == 0xD0)
		key+=4;
	else if (rows == 0x70)
		key+=0;
	lastkey = key;
	return lastkey;
}
