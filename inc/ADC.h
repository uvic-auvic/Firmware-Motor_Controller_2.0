#ifndef ADC_H_
#define ADC_H_

#define SENSOR_QUANTITY 17
#define AMMOUNT_OF_RECORDED_VALUES 5

extern uint16_t ADC_Values[SENSOR_QUANTITY][AMMOUNT_OF_RECORDED_VALUES];
extern uint8_t Read_Position;
extern uint8_t ADC_count;
double average;

typedef enum ADC_sensors{
	//current
		Curr_ADC1 = 0,
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

extern void set_ADC_channel(ADC_sensors_t ADC_sensor_x);

extern void set_motor_current_temp_MUX(ADC_sensors_t ADC_sensor_x);

extern void read_ADC(ADC_sensors_t ADC_sensor_x);

extern uint16_t return_ADC_value(ADC_sensors_t ADC_sensor_x);

extern void ADCTask(void *dummy);

#endif /* ADC_H_ */
