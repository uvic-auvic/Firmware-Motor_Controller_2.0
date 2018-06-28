/*
 * I2C.h
 *
 *  Created on: Jun 4, 2018
 *      Author: rober
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t I2C_mutex;

extern uint16_t switch_endiness_uint16(uint16_t input);

extern uint32_t switch_endiness_uint32(uint32_t input, uint8_t numBytes);

extern void I2C_setup();

extern void I2C_read(uint8_t address, uint8_t numBytes, uint8_t *message);

extern void I2C_write(uint8_t address, uint8_t numBytes, uint8_t message[]);

#endif /* I2C_H_ */
