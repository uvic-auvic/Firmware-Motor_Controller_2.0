#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "motors.h"

static void initTempCurrMUX(){
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
	GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;;
	GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIOD_InitStruct);
}

uint8_t get_motor_current(enum motors motor_x){
	switch (motor_x){
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
			break;
		}
}

uint8_t get_motor_temp(enum motors motor_x){
	switch (motor_x){
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
			break;
	}
}
