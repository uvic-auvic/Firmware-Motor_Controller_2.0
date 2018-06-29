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
#include "INA226_Current_Sensor.h"
#include "Internal_Pressure_Sensor.h"
#include "SI7021_temp_humidity_sensor.h"
#include "I2C.h"

//Global variables for I2C sensors values
uint32_t supply_current = 0;
uint16_t temperature = 0;
uint16_t humidity = 0;
uint32_t internalPressure = 0;

static void update_I2C_Sensors(void *dummy) {

	//Init function for I2C sensors will go here.
	//Temperature sensor and INA226 Current sensor will have one
    init_INA226_Current_Sensor();
    init_internal_presure_sensor();

	while(1) {
		//Update functions for I2C sensors will go here.
		//All I2C sensors should have one
		supply_current = update_system_current();
		internalPressure = Update_Internal_Pressure();
		humidity = Update_Humidity();
		temperature = Update_Temperature_From_Last_Reading();
		vTaskDelay(1000);
	}
}

extern void init_I2C_Sensors() {
	I2C_setup();
	// Create the FSM task
    xTaskCreate(update_I2C_Sensors,
		(const signed char *)"update_I2C_Sensors",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */
}
