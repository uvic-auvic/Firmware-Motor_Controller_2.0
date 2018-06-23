/*
 * I2C.h
 *
 *  Created on: Jun 4, 2018
 *      Author: rober
 */

#ifndef I2C_H_
#define I2C_H_

typedef enum{
	read,
	write,
	nothing
} I2C_state_t;

extern void I2C_setup();

extern void I2C_read(uint8_t address, uint8_t numBytes, uint8_t *message);

extern void I2C_write(uint8_t address, uint8_t numBytes, uint8_t message[]);

#endif /* I2C_H_ */
