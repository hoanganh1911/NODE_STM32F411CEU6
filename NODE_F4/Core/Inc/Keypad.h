/*
 * keypad_pcf.h
 *
 *  Created on: Mar 4, 2023
 *      Author: hoanganh
 */

#ifndef KEYPAD_PCF_H_
#define KEYPAD_PCF_H_
/*----------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
/*----------------------------------------*/
#define PCF_ADDR 0x20
/*----------------------------------------*/
uint8_t getkey(I2C_HandleTypeDef *_hi2c);
#endif /* KEYPAD_PCF_H_ */
