/*
 * I2C_Sensors.h
 *
 *  Created on: Jun 22, 2018
 *      Author: Poornachander
 */

#ifndef I2C_SENSORS_H_
#define I2C_SENSORS_H_

extern uint32_t supply_current;
extern uint16_t temperature;
extern uint16_t humidity;
extern uint32_t internalPressure;

extern void init_I2C_Sensors();

#endif /* I2C_SENSORS_H_ */
