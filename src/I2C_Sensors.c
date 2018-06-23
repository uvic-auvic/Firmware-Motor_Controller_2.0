/*
 * I2C_Sensors.c
 *
 *  Created on: Jun 22, 2018
 *      Author: Poornachander
 */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

//Global variables for I2C sensors values
uint32_t supply_current = 0;
uint16_t temperature = 0;
uint16_t humidity = 0;
uint32_t internalPressure = 123456;

static void update_I2C_Sensors() {

	//Init function for I2C sensors will go here.
	//Temperature sensor and INA226 Current sensor will have one

	while(1) {
		//Update functions for I2C sensors will go here.
		//All I2C sensors should have one


	}

}

extern void init_I2C_Sensors() {

	// Create the FSM task
    xTaskCreate(update_I2C_Sensors,
		(const char *)"update_I2C_Sensors",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */

}
