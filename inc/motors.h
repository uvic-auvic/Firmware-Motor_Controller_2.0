#ifndef MOTORS_H-
#define MOTORS_H_

enum motors{
//current
	Motor_Curr_ADC1,
	Motor_Curr_ADC2,
	Motor_Curr_ADC3,
	Motor_Curr_ADC4,
	Motor_Curr_ADC5,
	Motor_Curr_ADC6,
	Motor_Curr_ADC7,
	Motor_Curr_ADC8,
//temperature
	Motor_Temp_ADC1,
	Motor_Temp_ADC2,
	Motor_Temp_ADC3,
	Motor_Temp_ADC4,
	Motor_Temp_ADC5,
	Motor_Temp_ADC6,
	Motor_Temp_ADC7,
	Motor_Temp_ADC8
};

uint8_t get_motor_current(enum motors motor_x);

uint8_t get_motor_temp(enum motors motor_x);

#endif
