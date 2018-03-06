#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "motors.h"

void enable_outputs(){
	//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//GPIOC Configuration
	 GPIO_InitTypeDef GPIOC_InitStruct;
	  GPIOC_InitStruct.GPIO_Pin = 0x380;
	  GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	  GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	  GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOC, &GPIOC_InitStruct);
	 //GPIOD Configuration
	  GPIO_InitTypeDef GPIOD_InitStruct;
	   GPIOD_InitStruct.GPIO_Pin = 0x380;
	   GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	   GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	   GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	   GPIO_Init(GPIOD, &GPIOD_InitStruct);
}

uint8_t get_motor_current(enum motors motor_x){
	switch (motor_x){
	case Motor_Curr_ADC1:
		GPIOC-> BSRRL &= ~(0x380); //turn off PC7-PC9
		break;
	case Motor_Curr_ADC2:
		GPIOC-> BSRRL |= 0x80; //turn on PC7
		break;
	case Motor_Curr_ADC3:
		GPIOC-> BSRRL |= 0x100; //turn on PC8
		break;
	case Motor_Curr_ADC4:
		GPIOC-> BSRRL |= 0x180; //turn on PC8 and PC7
		break;
	case Motor_Curr_ADC5:
		GPIOC-> BSRRL |= 0x200; //turn on PC9
		break;
	case Motor_Curr_ADC6:
		GPIOC-> BSRRL |= 0x280; //turn on PC9 and PC7
		break;
	case Motor_Curr_ADC7:
		GPIOC-> BSRRL |= 300; //turn on PC9 and PC8
		break;
	case Motor_Curr_ADC8:
		GPIOC-> BSRRL |= 0x380; //turn on PC7-PC9
		break;
	default:
		break;
	}
}

uint8_t get_motor_temp(enum motors motor_x){
	switch (motor_x){
	case Motor_Temp_ADC1:
		GPIOD-> BSRRL &= ~(0x380); //turn off PD7-PD9
		break;
	case Motor_Temp_ADC2:
		GPIOD-> BSRRL |= 0x80; //turn on PD7
		break;
	case Motor_Temp_ADC3:
		GPIOD-> BSRRL |= 0x100; //turn on PD8
		break;
	case Motor_Temp_ADC4:
		GPIOD-> BSRRL |= 0x180; //turn on PD7 and PD8
		break;
	case Motor_Temp_ADC5:
		GPIOD-> BSRRL |= 0x200; //turn on PD9
		break;
	case Motor_Temp_ADC6:
		GPIOD-> BSRRL |= 0x280; //turn on PD9 and PD7
		break;
	case Motor_Temp_ADC7:
		GPIOD-> BSRRL |= 300; //turn on PD9 and PD8
		break;
	case Motor_Temp_ADC8:
		GPIOD-> BSRRL |= 0x380; //turn on PD7-PD9
		break;
	default:
		break;
	}
}
