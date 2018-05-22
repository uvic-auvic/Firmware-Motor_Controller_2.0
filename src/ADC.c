/*
 * ADC.c
 *
 *  Created on: May 18, 2018
 *      Author: rober
 */
#include "stm32f4xx.h"
#include "ADC.h"
#include "motors.h"

static void Enable_ADC(select_ADC_t select_ADC_x){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//GPIOC Setup
	GPIO_InitTypeDef GPIOC_InitStruct;
	GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOC_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIOC_InitStruct);

	//ADC1 Setup
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);

	//NVIC Setup
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0X0F;
	NVIC_Init(&NVIC_InitStruct);
	//Enable ADC end of conversion flag at the end of each channel conversion
	ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

	//Triggers an interrupt at each EOC flag
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	//Select ADC channels
	switch(select_ADC_x){
		case temperature:
			ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_56Cycles); //PC3 Temperature
			break;
		case current:
			ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_56Cycles); //PC2 Current
			break;
		case water:
			ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_56Cycles); //PC1 Water
			break;
	}
	//Enable ADC
	ADC1->CR2 |= ADC_CR2_ADON;
}

extern void read_ADC(motor_sensors_t motor_sensor_x){
	switch (motor_sensor_x) {
	//read current
		case Motor_Curr_ADC1:
			Read_Position = 0;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC2:
			Read_Position = 1;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC3:
			Read_Position = 2;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC4:
			Read_Position = 3;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC5:
			Read_Position = 4;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC6:
			Read_Position = 5;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC7:
			Read_Position = 6;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Curr_ADC8:
			Read_Position = 7;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		//read Temperature
		case Motor_Temp_ADC1:
			Read_Position = 8;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC2:
			Read_Position = 9;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC3:
			Read_Position = 10;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC4:
			Read_Position = 11;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC5:
			Read_Position = 12;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC6:
			Read_Position = 13;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC7:
			Read_Position = 14;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Temp_ADC8:
			Read_Position = 15;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		//read pressure
		case Motor_Water_ADC1:
			Read_Position = 16;
			Enable_ADC(water);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Motor_Water_ADC2:
			Read_Position = 17;
			Enable_ADC(water);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		}
}

void ADC_IRQHandler(void){
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	ADC_Values[Read_Position][ADC_count] = ADC1->DR;
    if(ADC_count >= 4){
    	for(ADC_count = 0; ADC_count < 5; ADC_count++){
    		average += ADC_Values[Read_Position][ADC_count];
    	}
 		average /= 5;
    }else{
    	ADC_count++;
    	ADC_SoftwareStartConv(ADC1);
    }
}
