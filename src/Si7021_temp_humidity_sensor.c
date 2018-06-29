#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "I2C.h"
#include "semphr.h"
#include "debug.h"

//Time out
#define I2C_TIMEOUT                  2000

//Si7021 register commands
#define CMD_MEASURE_RH_HM            0xE5
#define CMD_MEASURE_RH_NM            0xF5
#define CMD_MEASURE_TEMP_HM          0xE3
#define CMD_MEASURE_TEMP_NH          0xF3
#define CMD_READ_TEMP_PREV_RH        0xE0
#define CMD_RESET                    0xFE

//Si7021 I2C slave addresses
#define Si_Address                   0b1000000


//Take the relative humidity and return it as a percentage
extern uint16_t Update_Humidity() {
		uint8_t humidityAddress = CMD_MEASURE_RH_HM;

		I2C_write(Si_Address , 1, &humidityAddress);
		if (ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT) == 0) {
			debug_write("I2C_TIMEOUT\n");
		} else {
			debug_write("WRITE_SUCESS\n");
		}

		uint16_t relativeHumidity = 0;

		I2C_read(Si_Address, 2, (uint8_t *)&relativeHumidity);
		if (ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT) == 0) {
			debug_write("I2C_TIMEOUT\r\n");
		} else {
			debug_write("WRITE_SUCESS\r\n");
		}

		char temp[6] = {};
		itoa(relativeHumidity, temp, 10);
		UART_push_out(temp);

		relativeHumidity = switch_endiness_uint16(relativeHumidity);
		relativeHumidity = ((125*relativeHumidity)/65536)-6;

		return relativeHumidity; //returns relative humidity %
}
// take the temperature in Kelvins and return it as 10*(actual temperature)
extern uint16_t Update_Temperature() {

		uint8_t tempAddress = CMD_MEASURE_TEMP_HM;

		I2C_write(Si_Address, 1, &tempAddress);
		ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

		uint16_t temperature = 0;

		I2C_read(Si_Address, 2, (uint8_t *)&temperature);
		ulTaskNotifyTake(pdTRUE, I2C_TIMEOUT);

		temperature = switch_endiness_uint16(temperature);
		temperature = ((1757.2*temperature)/65536)+2263;

		return temperature; //returns 10*temperature in Kelvins
}
