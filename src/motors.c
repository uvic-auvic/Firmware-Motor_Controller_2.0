#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "motors.h"

/* Initialization functions
 * ----------------------------------------------------------
 */
static void init_motor_current_temp_MUX(){
	//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//GPIOC Configuration
	GPIO_InitTypeDef GPIOC_InitStruct;
	GPIOC_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIOC_InitStruct);
	//GPIOD Configuration
	GPIO_InitTypeDef GPIOD_InitStruct;
	GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIOD_InitStruct);
}

static void init_motor_speed_control()
{
	//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//init timers
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//GPIOA Configuration for motors 1-4 and 7
	GPIO_InitTypeDef GPIOA_InitStruct;
	GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_6;
	GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOA_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOA_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIOA_InitStruct);
	//GPIOD configuration for motors 5-6
	GPIO_InitTypeDef GPIOD_InitStruct;
	GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12;
	GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIOD_InitStruct);
	//GPIOB configuration for motor 8
	GPIO_InitTypeDef GPIOB_InitStruct;
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);
	//pin alternate function timer configuration
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	//timer set up
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 2) / 21000000) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//pwm init
	TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //pwm config for channel 1 PA0 motor 1
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    //pwm config for channel 4 PA3 motor 2
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    //pwm config for channel 3 PA10 motor 3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    //pwm config for channel 1 PA8 motor 4
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    //pwm config for channel 4 PD15 motor 5
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    //pwm config for channel 1 PD12 motor 6
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    //pwm config for channel 1 PA6 motor 7
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    //pwm config for channel 3 PB0 motor 8
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//additional code for TIM1
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

static void init_motor_speed_feedback()
{

}

/* Extern declarations in each section must go below static declarations */
extern void init_motors()
{
	/* Initialize sub-modules */
	init_motor_current_temp_MUX();
	init_motor_speed_control();
	init_motor_speed_feedback();
}

/* Robert's section.
 * Setting motor speed.
 * ----------------------------------------------------------
 */

/* Put any static/hidden functions your code may need below here: */

/* Robert's extern/visible functions go below here: */
extern void motor_set_speed_percent(motors_t motor_x, uint8_t speed, direction_t dir)
{
	if(dir == Reverse) speed *= -1;
	switch(motor_x){
		case Motor1:
			TIM_SetCompare1(TIM2, speed*10);
			break;
		case Motor2:
			TIM_SetCompare4(TIM2, speed*10);
			break;
		case Motor3:
			TIM_SetCompare3(TIM1, speed*10);
			break;
		case Motor4:
			TIM_SetCompare1(TIM1, speed*10);
			break;
		case Motor5:
			TIM_SetCompare4(TIM4, speed*10);
			break;
		case Motor6:
			TIM_SetCompare1(TIM4, speed*10);
			break;
		case Motor7:
			TIM_SetCompare1(TIM3, speed*10);
			break;
		case Motor8:
			TIM_SetCompare3(TIM3, speed*10);
			break;
	}

}

/* Poorna's section.
 * Reading RPM.
 * ----------------------------------------------------------
 */

/* Put any static/hidden functions your code may need below here: */

/* Poorna's extern/visible functions go below here: */
extern int16_t motor_get_rpm(motors_t motor_x)
{
	return 0;
}

/* Code from Issue #8 (by Robert Keen).
 * Reading MUX'ed sensor data.
 * ----------------------------------------------------------
 */

extern uint8_t get_motor_current(motor_sensors_t motor_sensor_x){
	switch (motor_sensor_x){
	case Motor_Curr_ADC1:
			GPIOC-> BSRRL &= ~(GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//turn off PC7-PC9
			return 0;
			break;
		case Motor_Curr_ADC2:
			GPIOC-> BSRRL |= GPIO_Pin_7;//turn on PC7
			return 0;
			break;
		case Motor_Curr_ADC3:
			GPIOC-> BSRRL |= GPIO_Pin_8; //turn on PC8
			return 0;
			break;
		case Motor_Curr_ADC4:
			GPIOC-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8; //turn on PC7-PC8
			return 0;
			break;
		case Motor_Curr_ADC5:
			GPIOC-> BSRRL |= GPIO_Pin_9; //turn on PC9
			return 0;
			break;
		case Motor_Curr_ADC6:
			GPIOC-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_9; //turn on PC7-PC9
			return 0;
			break;
		case Motor_Curr_ADC7:
			GPIOC-> BSRRL |= GPIO_Pin_8 | GPIO_Pin_9; //turn on PC8-PC9
			return 0;
			break;
		case Motor_Curr_ADC8:
			GPIOC-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //turn on PC7-PC9
			return 0;
			break;
		default:
			return 0;
			break;
		}
}

extern uint8_t get_motor_temp(motor_sensors_t motor_sensor_x){
	switch (motor_sensor_x){
	case Motor_Temp_ADC1:
			GPIOD-> BSRRL &= ~(GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//turn off PD7-PD9
			return 0;
			break;
		case Motor_Temp_ADC2:
			GPIOD-> BSRRL |= GPIO_Pin_7;//turn on PD7
			return 0;
			break;
		case Motor_Temp_ADC3:
			GPIOD-> BSRRL |= GPIO_Pin_8; //turn on PD8
			return 0;
			break;
		case Motor_Temp_ADC4:
			GPIOD-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8; //turn on PD7-PD8
			return 0;
			break;
		case Motor_Temp_ADC5:
			GPIOD-> BSRRL |= GPIO_Pin_9; //turn on PD9
			return 0;
			break;
		case Motor_Temp_ADC6:
			GPIOD-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_9; //turn on PD7-PD9
			return 0;
			break;
		case Motor_Temp_ADC7:
			GPIOD-> BSRRL |= GPIO_Pin_8 | GPIO_Pin_9; //turn on PD8-PD9
			return 0;
			break;
		case Motor_Temp_ADC8:
			GPIOD-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //turn on PD7-PD9
			return 0;
			break;
		default:
			return 0;
			break;
	}
}
