/*
 * ADC.h
 *
 *  Created on: May 18, 2018
 *      Author: rober
 */

#ifndef ADC_H_
#define ADC_H_

uint16_t ADC_Values[18][5];
uint8_t Read_Position;
uint8_t ADC_count;
uint16_t average;

typedef enum ADC_sensors{
	//current
		Curr_ADC1 = 1,
		Curr_ADC2,
		Curr_ADC3,
		Curr_ADC4,
		Curr_ADC5,
		Curr_ADC6,
		Curr_ADC7,
		Curr_ADC8,
	//temperature
		Temp_ADC1,
		Temp_ADC2,
		Temp_ADC3,
		Temp_ADC4,
		Temp_ADC5,
		Temp_ADC6,
		Temp_ADC7,
		Temp_ADC8,
	//water
		Water_ADC1,
		Water_ADC2
} ADC_sensors_t;

typedef enum select_ADC {
	temperature,
	current,
	water
} select_ADC_t;

typedef enum {
	off,
	on
}find_all_mode_t;

typedef struct {
	uint16_t motor_1;
	uint16_t motor_2;
	uint16_t motor_3;
	uint16_t motor_4;
	uint16_t motor_5;
	uint16_t motor_6;
	uint16_t motor_7;
	uint16_t motor_8;
} motor_temperature_t;

typedef struct {
	uint16_t motor_1;
	uint16_t motor_2;
	uint16_t motor_3;
	uint16_t motor_4;
	uint16_t motor_5;
	uint16_t motor_6;
	uint16_t motor_7;
	uint16_t motor_8;
} motor_current_t;

typedef struct {
	uint16_t sensor_1;
	uint16_t sensor_2;
} pressure_t;

motor_current_t motor_current;
motor_temperature_t motor_temperature;
pressure_t pressure;

find_all_mode_t find_all_mode;
ADC_sensors_t ADC_sensor;

static void Enable_ADC(select_ADC_t select_ADC_x);

static uint16_t calculate_average(uint8_t Read_Position_x);

extern void read_ADC(ADC_sensors_t ADC_sensor_x);

static void store_average_ADC_value(ADC_sensors_t ADC_sensor_x);

static void store_all_average_ADC_values();

extern void find_all_ADC_values();

extern uint16_t return_ADC_value(ADC_sensors_t ADC_sensor_x);

#endif /* ADC_H_ */
