#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "motors.h"

void initADC_MUX(){
//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//GPIOC Configuration for multiplex control
	 GPIO_InitTypeDef GPIOC_InitStruct;

	  GPIOC_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	  GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	  GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	  GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOC, &GPIOC_InitStruct);

//GPIOC Configuration for ADC
	  ADC_InitTypeDef ADC_InitStruct;
	  ADC_CommonInitTypeDef ADC_CommonStruct;
	  ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
	  ADC_Init(ADC1, &ADC_InitStruct);
	  ADC_Cmd(ADC1, ENABLE);

//GPIOD Configuration for multiplex control
	  GPIO_InitTypeDef GPIOD_InitStruct;
	   GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;;
	   GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	   GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	   GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	   GPIO_Init(GPIOD, &GPIOD_InitStruct);
}

uint8_t get_motor_current(enum motors motor_x){
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_56Cycles);
	int current = 0;
	switch (motor_x){
	int ADC_Value;
	case Motor_Curr_ADC1:
			GPIOC-> BSRRL &= ~(GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//turn off PC7-PC9
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC2:
			GPIOC-> BSRRL |= GPIO_Pin_7;//turn on PC7
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC3:
			GPIOC-> BSRRL |= GPIO_Pin_8; //turn on PC8
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC4:
			GPIOC-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8; //turn on PC7-PC8
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC5:
			GPIOC-> BSRRL |= GPIO_Pin_9; //turn on PC9
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC6:
			GPIOC-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_9; //turn on PC7-PC9
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC7:
			GPIOC-> BSRRL |= GPIO_Pin_8 | GPIO_Pin_9; //turn on PC8-PC9
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Curr_ADC8:
			GPIOC-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //turn on PC7-PC9
			current = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		default:
			break;
		}
}

uint8_t get_motor_temp(enum motors motor_x){
	switch (motor_x){
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_56Cycles);
	int temperature = 0;
	case Motor_Temp_ADC1:
			GPIOD-> BSRRL &= ~(GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//turn off PD7-PD9
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC2:
			GPIOD-> BSRRL |= GPIO_Pin_7;//turn on PD7
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC3:
			GPIOD-> BSRRL |= GPIO_Pin_8; //turn on PD8
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC4:
			GPIOD-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8; //turn on PD7-PD8
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC5:
			GPIOD-> BSRRL |= GPIO_Pin_9; //turn on PD9
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC6:
			GPIOD-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_9; //turn on PD7-PD9
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC7:
			GPIOD-> BSRRL |= GPIO_Pin_8 | GPIO_Pin_9; //turn on PD8-PD9
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		case Motor_Temp_ADC8:
			GPIOD-> BSRRL |= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //turn on PD7-PD9
			temperature = ADC_GetConversionValue(ADC1);
			return 0;
			break;
		default:
			break;
	}
}
