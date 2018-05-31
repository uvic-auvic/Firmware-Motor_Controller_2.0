#include "stm32f4xx.h"
#include "ADC.h"
#include "motors.h"

//ADC_sensors_t ADC_sensor = Curr_ADC1;
motor_sensors_t motor_sensor = Motor_Curr_ADC1;


extern void init_ADC(){
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
	ADC_InitStruct.ADC_NbrOfConversion = 2;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);

	//NVIC Setup
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_Init(&NVIC_InitStruct);

	//Enable ADC end of conversion flag at the end of each channel conversion
	ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

	//Triggers an interrupt at each EOC flag
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}

static void Enable_ADC(ADC_sensors_t ADC_sensor_x){
	//Select ADC channels
	if(((ADC_sensor_x <= Curr_ADC4) && (ADC_sensor_x >= Curr_ADC1)) || ((ADC_sensor_x <= Temp_ADC4) && (ADC_sensor_x >= Temp_ADC1))){
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_56Cycles); //PC3
	}
	else if(((ADC_sensor_x <= Curr_ADC8) && (ADC_sensor_x >= Curr_ADC5)) || ((ADC_sensor_x <= Temp_ADC8) && (ADC_sensor_x >= Temp_ADC5))){
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_56Cycles); //PC2
	}
	else if(ADC_sensor_x == Water_ADC){
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_56Cycles); //PC1
	}
	//Enable ADC
	ADC1->CR2 |= ADC_CR2_ADON;
}

static uint16_t double_to_int(double double_x){
	uint16_t double_int = (uint16_t)double_x;
	if((double_x - (double)double_int) >= 0.5) double_int++;
	return double_int;
}

static double calculate_average(uint8_t Read_Position_x){
	average = 0;
	for(ADC_count = 0; ADC_count < 5; ADC_count++){
	    average += (double)ADC_Values[Read_Position_x][ADC_count];
	 }
	average /= 5;
	return average;
}

extern void read_ADC(ADC_sensors_t ADC_sensor_x){
	switch (ADC_sensor_x) {
	//read current
		case Curr_ADC1:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC1);
			Read_Position = 0;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC2:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC2);
			Read_Position = 1;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC3:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC3);
			Read_Position = 2;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC4:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC4);
			Read_Position = 3;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC5:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC5);
			Read_Position = 4;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC6:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC6);
			Read_Position = 5;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC7:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC7);
			Read_Position = 6;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Curr_ADC8:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Curr_ADC8);
			Read_Position = 7;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		//read Temperature
		case Temp_ADC1:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC1);
			Read_Position = 8;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC2:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC2);
			Read_Position = 9;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC3:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC3);
			Read_Position = 10;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC4:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC4);
			Read_Position = 11;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC5:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC5);
			Read_Position = 12;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC6:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC6);
			Read_Position = 13;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC7:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC7);
			Read_Position = 14;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		case Temp_ADC8:
			set_motor_current_temp_MUX(motor_sensor);
			//set_motor_current_temp_MUX(Motor_Temp_ADC8);
			Read_Position = 15;
			Enable_ADC(ADC_sensor_x);
			ADC_count = 0;
			ADC_SoftwareStartConv(ADC1);
			break;
		//read pressure
		case Water_ADC:
			Read_Position = 16;
			Enable_ADC(ADC_sensor_x);
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
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC2:
			Read_Position = 1;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC3:
			Read_Position = 2;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC4:
			Read_Position = 3;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC5:
			Read_Position = 4;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC6:
			Read_Position = 5;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC7:
			Read_Position = 6;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		case Curr_ADC8:
			Read_Position = 7;
			average = calculate_average(Read_Position)*2.15;
			return double_to_int(average);
			break;
		//read Temperature
		case Temp_ADC1:
			Read_Position = 8;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC2:
			Read_Position = 9;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC3:
			Read_Position = 10;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC4:
			Read_Position = 11;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC5:
			Read_Position = 12;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC6:
			Read_Position = 13;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC7:
			Read_Position = 14;
			return calculate_average(Read_Position)*0.0177;
			break;
		case Temp_ADC8:
			Read_Position = 15;
			return calculate_average(Read_Position)*0.0177;
			break;
		//read pressure
		case Water_ADC:
			Read_Position = 16;
			return calculate_average(Read_Position)*0.088;
			break;
		}
}

void ADC_IRQHandler(void){
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	ADC_Values[Read_Position][ADC_count] = ADC1->DR;
	if(ADC_count < 5){
		ADC_count++;
		ADC_SoftwareStartConv(ADC1);
	}
}
