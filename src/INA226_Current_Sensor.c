/*
 * INA226_Current_Sensor.c
 *
 *  Created on: May 29, 2018
 *      Author: Poornachander
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "I2C.h"

#define I2C_TIMEOUT		2000

//INA226 Current and Power Sensor Register address
#define CONFIG_REG_ADDRESS	0x00
#define CALIB_REG_ADDRESS	0x05
#define CURRENT_REG_ADDRESS	0x04
#define POWER_REG_ADDRESS	0x03
#define BUS_VOLTAGE_REG_ADDRESS	0x02
#define SHUNT_VOLTAGE_REG_ADDRESS	0x01

//INA226 Slave address
#define MOTOR_SENSOR_I2C_ADDRESS	0b01001111 //0x4F
#define SYSTEM_SENSOR_I2C_ADDRESS	0b01001010 //0x4A

//Calibration register value
#define CALIB_REG_VALUE_HIGH_BYTE	0x10 //4mA per bit
#define CALIB_REG_VALUE_LOW_BYTE	0xAA //4mA per bit

//Correction to take into account trace resistance
#define SYSTEM_CURRENT_CALC_CORRECTION	1 //Needs to be calibrated
#define MOTOR_CURRENT_CALC_CORRECTION	1 //Needs to be calibrated

//Formulas to convert bits to actual values with units
#define TO_CURRENT(x) ((x)*(4))
#define TO_VOLTAGE(x) ((x)*1.25)
#define TO_SHUNT_VOLTAGE(x) ((x)*2.5)
#define TO_POWER(x) ((x)*25)

extern void init_INA226_Current_Sensor() {

	uint8_t valueToWriteOnStartup[3] = {CALIB_REG_ADDRESS, CALIB_REG_VALUE_HIGH_BYTE, CALIB_REG_VALUE_LOW_BYTE};

	I2C_write(SYSTEM_SENSOR_I2C_ADDRESS, 3, valueToWriteOnStartup);

	I2C_write(MOTOR_SENSOR_I2C_ADDRESS, 3, valueToWriteOnStartup);


}

extern uint32_t update_system_current() {


	uint8_t currentReg = CURRENT_REG_ADDRESS;

	I2C_write(SYSTEM_SENSOR_I2C_ADDRESS, 1, &currentReg);

	uint32_t current = 0;

	I2C_read(SYSTEM_SENSOR_I2C_ADDRESS, 2, (uint8_t *)&current);

	current = switch_endiness_uint16(current);
	current = TO_CURRENT(current) * (SYSTEM_CURRENT_CALC_CORRECTION);

	return current;


}

extern uint32_t update_motor_current() {

	uint8_t currentReg = CURRENT_REG_ADDRESS;

	I2C_write(MOTOR_SENSOR_I2C_ADDRESS, 1, &currentReg);

	uint32_t current = 0;

	I2C_read(MOTOR_SENSOR_I2C_ADDRESS, 2, (uint8_t *)&current);

	current = switch_endiness_uint16(current);
	current = TO_CURRENT(current) * (MOTOR_CURRENT_CALC_CORRECTION);

	return current;

}

extern uint16_t get_system_bus_voltage() {

	uint8_t voltageReg = BUS_VOLTAGE_REG_ADDRESS;

	I2C_write(SYSTEM_SENSOR_I2C_ADDRESS, 1, &voltageReg);

	uint16_t voltage = 0;

	I2C_read(SYSTEM_SENSOR_I2C_ADDRESS, 2, (uint8_t *)&voltage);

	voltage = switch_endiness_uint16(voltage);
	voltage = TO_VOLTAGE(voltage);

	return voltage;

}

extern uint16_t get_motor_bus_voltage() {

	uint8_t voltageReg = BUS_VOLTAGE_REG_ADDRESS;

	I2C_write(MOTOR_SENSOR_I2C_ADDRESS, 1, &voltageReg);

	uint16_t voltage = 0;

	I2C_read(MOTOR_SENSOR_I2C_ADDRESS, 2, (uint8_t *)&voltage);

	voltage = switch_endiness_uint16(voltage);
	voltage = TO_VOLTAGE(voltage);

	return voltage;

}

extern uint16_t get_system_shunt_voltage() {


	uint8_t shuntVoltageReg = SHUNT_VOLTAGE_REG_ADDRESS;

	I2C_write(SYSTEM_SENSOR_I2C_ADDRESS, 1, &shuntVoltageReg);

	uint16_t shuntVoltage = 0;

	I2C_read(SYSTEM_SENSOR_I2C_ADDRESS, 2, (uint8_t *)&shuntVoltage);

	shuntVoltage = switch_endiness_uint16(shuntVoltage);
	shuntVoltage = TO_SHUNT_VOLTAGE(shuntVoltage) * (SYSTEM_CURRENT_CALC_CORRECTION);

	return shuntVoltage;

}

extern uint16_t get_motor_shunt_voltage() {

	uint8_t shuntVoltageReg = SHUNT_VOLTAGE_REG_ADDRESS;

	I2C_write(MOTOR_SENSOR_I2C_ADDRESS, 1, &shuntVoltageReg);

	uint16_t shuntVoltage = 0;

	I2C_read(MOTOR_SENSOR_I2C_ADDRESS, 2, (uint8_t *)&shuntVoltage);

	shuntVoltage = switch_endiness_uint16(shuntVoltage);
	shuntVoltage = TO_SHUNT_VOLTAGE(shuntVoltage) * (MOTOR_CURRENT_CALC_CORRECTION);

	return shuntVoltage;

}
