/*
 * ADC.c
 *
 *  Created on: May 18, 2018
 *      Author: rober
 */
#include "stm32f4xx.h"
#include "ADC.h"

find_all_mode_t find_all_mode = off;
ADC_sensors_t ADC_sensor = Curr_ADC1;

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

static uint16_t calculate_average(uint8_t Read_Position_x){
	for(ADC_count = 0; ADC_count < 5; ADC_count++){
	    average += ADC_Values[Read_Position_x][ADC_count];
	 }
	average /= 5;
	return average;
}

extern void read_ADC(ADC_sensors_t ADC_sensor_x){
	switch (ADC_sensor_x) {
	//read current
		case Curr_ADC1:
			Read_Position = 0;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC2:
			Read_Position = 1;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC3:
			Read_Position = 2;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC4:
			Read_Position = 3;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC5:
			Read_Position = 4;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC6:
			Read_Position = 5;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC7:
			Read_Position = 6;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC8:
			Read_Position = 7;
			Enable_ADC(current);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		//read Temperature
		case Temp_ADC1:
			Read_Position = 8;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC2:
			Read_Position = 9;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC3:
			Read_Position = 10;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC4:
			Read_Position = 11;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC5:
			Read_Position = 12;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC6:
			Read_Position = 13;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC7:
			Read_Position = 14;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC8:
			Read_Position = 15;
			Enable_ADC(temperature);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		//read pressure
		case Water_ADC1:
			Read_Position = 16;
			Enable_ADC(water);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Water_ADC2:
			Read_Position = 17;
			Enable_ADC(water);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		}
}

extern uint16_t return_ADC_value(ADC_sensors_t ADC_sensor_x){
	switch (ADC_sensor_x) {
	//read current
		case Curr_ADC1:
			Read_Position = 0;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC2:
			Read_Position = 1;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC3:
			Read_Position = 2;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC4:
			Read_Position = 3;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC5:
			Read_Position = 4;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC6:
			Read_Position = 5;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC7:
			Read_Position = 6;
			return calculate_average(Read_Position);
			break;
		case Curr_ADC8:
			Read_Position = 7;
			return calculate_average(Read_Position);
			break;
		//read Temperature
		case Temp_ADC1:
			Read_Position = 8;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC2:
			Read_Position = 9;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC3:
			Read_Position = 10;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC4:
			Read_Position = 11;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC5:
			Read_Position = 12;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC6:
			Read_Position = 13;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC7:
			Read_Position = 14;
			return calculate_average(Read_Position);
			break;
		case Temp_ADC8:
			Read_Position = 15;
			return calculate_average(Read_Position);
			break;
		//read pressure
		case Water_ADC1:
			Read_Position = 16;
			return calculate_average(Read_Position);
			break;
		case Water_ADC2:
			Read_Position = 17;
			return calculate_average(Read_Position);
			break;
		}
}

static void store_average_ADC_value(ADC_sensors_t ADC_sensor_x){
	switch (ADC_sensor_x) {
	//read current
		case Curr_ADC1:
			Read_Position = 0;
			motor_current.motor_1 = calculate_average(Read_Position);
			break;
		case Curr_ADC2:
			Read_Position = 1;
			motor_current.motor_2 = calculate_average(Read_Position);
			break;
		case Curr_ADC3:
			Read_Position = 2;
			motor_current.motor_3 = calculate_average(Read_Position);
			break;
		case Curr_ADC4:
			Read_Position = 3;
			motor_current.motor_4 = calculate_average(Read_Position);
			break;
		case Curr_ADC5:
			Read_Position = 4;
			motor_current.motor_5 = calculate_average(Read_Position);
			break;
		case Curr_ADC6:
			Read_Position = 5;
			motor_current.motor_6 = calculate_average(Read_Position);
			break;
		case Curr_ADC7:
			Read_Position = 6;
			motor_current.motor_7 = calculate_average(Read_Position);
			break;
		case Curr_ADC8:
			Read_Position = 7;
			motor_current.motor_8 = calculate_average(Read_Position);
			break;
		//read Temperature
		case Temp_ADC1:
			Read_Position = 8;
			motor_temperature.motor_1 = calculate_average(Read_Position);
			break;
		case Temp_ADC2:
			Read_Position = 9;
			motor_temperature.motor_2 = calculate_average(Read_Position);
			break;
		case Temp_ADC3:
			Read_Position = 10;
			motor_temperature.motor_3 = calculate_average(Read_Position);
			break;
		case Temp_ADC4:
			Read_Position = 11;
			motor_temperature.motor_4 = calculate_average(Read_Position);
			break;
		case Temp_ADC5:
			Read_Position = 12;
			motor_temperature.motor_5 = calculate_average(Read_Position);
			break;
		case Temp_ADC6:
			Read_Position = 13;
			motor_temperature.motor_6 = calculate_average(Read_Position);
			break;
		case Temp_ADC7:
			Read_Position = 14;
			motor_temperature.motor_7 = calculate_average(Read_Position);
			break;
		case Temp_ADC8:
			Read_Position = 15;
			motor_temperature.motor_8 = calculate_average(Read_Position);
			break;
		//read pressure
		case Water_ADC1:
			Read_Position = 16;
			pressure.sensor_1 = calculate_average(Read_Position);
			break;
		case Water_ADC2:
			Read_Position = 17;
			pressure.sensor_2 = calculate_average(Read_Position);
			break;
		}
}

static void store_all_average_ADC_values(){
	for(ADC_sensors_t ADC_sensor_x = Curr_ADC1; ADC_sensor_x <= Water_ADC2; ADC_sensor_x++){
		store_average_ADC_value(ADC_sensor_x);
	}
}

extern void find_all_ADC_values(){
	find_all_mode = on;
	read_ADC(Curr_ADC1);
}

void ADC_IRQHandler(void){
	if(find_all_mode == off){
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		ADC_Values[Read_Position][ADC_count] = ADC1->DR;
		if(ADC_count < 5){
			ADC_count++;
			ADC_SoftwareStartConv(ADC1);
		}
	}
	else if(find_all_mode == on){
		while(ADC_sensor <= Water_ADC2){
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
			ADC_Values[Read_Position][ADC_count] = ADC1->DR;
			if(ADC_count < 5){
				ADC_count++;
				ADC_SoftwareStartConv(ADC1);
			}
			else{
				ADC_sensor++;
				if(ADC_sensor > Water_ADC2) break;
				else read_ADC(ADC_sensor);
			}

		}
	}
}
