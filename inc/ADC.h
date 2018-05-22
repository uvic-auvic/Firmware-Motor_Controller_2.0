/*
 * ADC.h
 *
 *  Created on: May 18, 2018
 *      Author: rober
 */

#ifndef ADC_H_
#define ADC_H_
#include "motors.h"

uint16_t ADC_Values[18][5];
uint8_t Read_Position;
uint8_t ADC_count;
uint16_t average;

typedef enum select_ADC {
	temperature,
	current,
	water
} select_ADC_t;

static void Enable_ADC(select_ADC_t select_ADC_x);

extern void read_ADC(motor_sensors_t motor_sensor_x);

#endif /* ADC_H_ */
