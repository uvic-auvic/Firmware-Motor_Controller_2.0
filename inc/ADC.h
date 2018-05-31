#ifndef ADC_H_
#define ADC_H_

uint16_t ADC_Values[17][5];
uint8_t Read_Position;
uint8_t ADC_count;
double average;

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
		Water_ADC
} ADC_sensors_t;

ADC_sensors_t ADC_sensor;

extern void init_ADC();

static void Enable_ADC(ADC_sensors_t ADC_sensor_x);

static double calculate_average(uint8_t Read_Position_x);

extern void read_ADC(ADC_sensors_t ADC_sensor_x);

extern uint16_t return_ADC_value(ADC_sensors_t ADC_sensor_x);

#endif /* ADC_H_ */
