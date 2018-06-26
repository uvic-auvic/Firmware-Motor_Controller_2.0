/*
 * Internal_Pressure_Sensor.c
 *
 *  Created on: Jun 19, 2018
 *      Author: Poornachander
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "I2C.h"
#include "semphr.h"

//Time out
#define I2C_TIMEOUT                  2000

//I2C Address and Commands
#define SENSOR_ADDRESS					0b1110111
#define PRESSURE_CONVERT_256_CMD		0x40 //Convert D1 (OSR=256)
#define PRESSURE_CONVERT_4096_CMD		0x48
#define TEMPERATURE_CONVERT_256_CMD		0x50
#define TEMPERATURE_CONVERT_4096_CMD	0x58
#define ADC_READ_REGISTER				0x00
#define PROM_C1_ADDRESS					0b10100010
#define PROM_C2_ADDRESS					0b10100100
#define PROM_C3_ADDRESS					0b10100110
#define PROM_C4_ADDRESS					0b10101000
#define PROM_C5_ADDRESS					0b10101010
#define PROM_C6_ADDRESS					0b10101100

//PROM registers
uint16_t promRegister[6] = {};

extern uint32_t Update_Internal_Pressure() {

	if (xSemaphoreTake(I2C_mutex, I2C_TIMEOUT) == pdTRUE) {

			//Start Pressure Conversion on the sensor
			uint8_t temp_cmd_var = PRESSURE_CONVERT_4096_CMD;
			I2C_write(SENSOR_ADDRESS , 1, &temp_cmd_var);
			ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

			vTaskDelay(15);

			//Set read register on the sensor
			temp_cmd_var = ADC_READ_REGISTER;
			I2C_write(SENSOR_ADDRESS, 1, &temp_cmd_var);
			ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

			//Read pressure value from sensor
			uint32_t internalPressure = 0;
			I2C_read(SENSOR_ADDRESS, 3, (uint8_t *)&internalPressure);
			ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

			//Start Temperature Conversion on the sensor
			temp_cmd_var = TEMPERATURE_CONVERT_4096_CMD;
			I2C_write(SENSOR_ADDRESS , 1, &temp_cmd_var);
			ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

			vTaskDelay(15);

			//Set read register on the sensor
			temp_cmd_var = ADC_READ_REGISTER;
			I2C_write(SENSOR_ADDRESS, 1, &temp_cmd_var);
			ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

			//Read temperature value from the sensors
			uint32_t temperature = 0;
			I2C_read(SENSOR_ADDRESS, 3, (uint8_t *)&temperature);
			ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

			xSemaphoreGive(I2C_mutex);

			internalPressure = switch_endiness_uint32(internalPressure, 3);
			temperature = switch_endiness_uint32(temperature, 3);

			// Do calculation to determine pressure
			int32_t dT = temperature - (promRegister[4] * 256);
			int64_t off = ((int64_t)promRegister[1] * 131072) + (double)((int64_t)promRegister[3] * dT)/(64);
			int64_t sens = ((int64_t)promRegister[0] * 65536) + (double)((int64_t)promRegister[2] * dT)/(128);
			internalPressure = (double)(((int64_t)internalPressure * (double)sens/2097152) - off)/(32768);

			return internalPressure; //returns relative humidity %
		} else {
			return 0xFFFF;
		}
}

extern void init_internal_presure_sensor() {

	if(xSemaphoreTake(I2C_mutex, I2C_TIMEOUT) == pdTRUE) {

		for(uint8_t i = 0; i < 6; i++) {

			uint8_t temp_cmd_var = PROM_C1_ADDRESS + (i*2);

			I2C_write(SENSOR_ADDRESS, 1, &temp_cmd_var);
			ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(I2C_TIMEOUT));

			I2C_read(SENSOR_ADDRESS, 2, (uint8_t *)&promRegister[i]);
			ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(I2C_TIMEOUT));
			promRegister[i] = switch_endiness_uint16(promRegister[i]);
		}

		xSemaphoreGive(I2C_mutex);
	}

}
