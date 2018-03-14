#ifndef MOTORS_H_
#define MOTORS_H_

typedef enum motors {
	Motor1 = 1,
	Motor2,
	Motor3,
	Motor4,
	Motor5,
	Motor6,
	Motor7,
	Motor8
} motors_t;

typedef enum direction {
	Forward = 'F',
	Reverse = 'R'
} direction_t;

typedef enum motor_sensors {
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
} motor_sensors_t;

extern void init_motors();

extern void motor_set_speed_percent(motors_t motor_x, uint8_t speed, direction_t dir);

extern int16_t motor_get_rpm(motors_t motor_x);

extern uint8_t get_motor_current(motor_sensors_t motor_sensor_x);

extern uint8_t get_motor_temp(motor_sensors_t motor_sensor_x);

#endif
