#include "stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ADC.h"
#include "stdlib.h"

uint16_t ADC_Values[SENSOR_QUANTITY][AMMOUNT_OF_RECORDED_VALUES];
uint8_t Read_Position;
uint8_t ADC_count;
ADC_sensors_t ADC_sensor = Curr_ADC1;

static void init_motor_current_temp_MUX() {
	//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//GPIOC Configuration
	GPIO_InitTypeDef GPIOC_InitStruct;
	GPIOC_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIOC_InitStruct);

	//GPIOD Configuration
	GPIO_InitTypeDef GPIOD_InitStruct;
	GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIOD_InitStruct);

	//GPIOB Configuration
	GPIO_InitTypeDef GPIOB_InitStruct;
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);
}

static void init_ADC_read(){
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

extern void init_ADC(){
	init_motor_current_temp_MUX();
	init_ADC_read();
	xTaskCreate(ADCTask,
		(const signed char *)"ADCTask",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */
}

extern void set_ADC_channel(ADC_sensors_t ADC_sensor_x){
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
	for(ADC_count = 0; ADC_count < AMMOUNT_OF_RECORDED_VALUES; ADC_count++){
	    average += (double)ADC_Values[Read_Position_x][ADC_count];
	 }
	average /= AMMOUNT_OF_RECORDED_VALUES;
	return average;
}

extern void set_motor_current_temp_MUX(ADC_sensors_t ADC_sensor_x) {
	switch (ADC_sensor_x) {
		case Curr_ADC1:
			GPIOC->ODR |= GPIO_Pin_9; //turn on PC9
			GPIOC->ODR &= ~(GPIO_Pin_7 | GPIO_Pin_8);
			break;
		case Curr_ADC2:
			GPIOC->ODR |= GPIO_Pin_9 | GPIO_Pin_8; //turn on PC9 and PC8
			GPIOC->ODR &= ~(GPIO_Pin_7);
			break;
		case Curr_ADC3:
			GPIOC->ODR |= GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7; //turn on PC9, PC8, and PC7
			break;
		case Curr_ADC4:
			GPIOC->ODR |= GPIO_Pin_9 | GPIO_Pin_7; //turn on PC9 and PC7
			GPIOC->ODR &= ~(GPIO_Pin_8);
			break;
		case Curr_ADC5:
			GPIOD->ODR |= GPIO_Pin_9; //turn on PD9
			GPIOD->ODR &= ~(GPIO_Pin_8);
			GPIOB->ODR &= ~(GPIO_Pin_15);
			break;
		case Curr_ADC6:
			GPIOD->ODR |= GPIO_Pin_9 | GPIO_Pin_8; //turn on PD9 and PD8
			GPIOB->ODR &= ~(GPIO_Pin_15);
			break;
		case Curr_ADC7:
			GPIOD->ODR |= GPIO_Pin_9 | GPIO_Pin_8; //turn on PD9 and PD8
			GPIOB->ODR |= GPIO_Pin_15; //turn on PB15
			break;
		case Curr_ADC8:
			GPIOD->ODR |= GPIO_Pin_9; //turn on PD9
			GPIOB->ODR |= GPIO_Pin_15; //turn on PB15
			GPIOD->ODR &= ~(GPIO_Pin_8);
			break;
		case Temp_ADC1:
			GPIOC->ODR |= GPIO_Pin_8; //turn on PC8
			GPIOC->ODR &= ~(GPIO_Pin_9 | GPIO_Pin_7);
			break;
		case Temp_ADC2:
			GPIOC->ODR |= GPIO_Pin_7; //turn on PC7
			GPIOC->ODR &= ~(GPIO_Pin_8 | GPIO_Pin_9);
			break;
		case Temp_ADC3:
			GPIOC->ODR &= ~(GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7); //turn off PC7-PC9
			break;
		case Temp_ADC4:
			GPIOC->ODR |= GPIO_Pin_8 | GPIO_Pin_7; //turn on PC7-PC8
			GPIOC->ODR &= ~(GPIO_Pin_9);
			break;
		case Temp_ADC5:
			GPIOD->ODR |= GPIO_Pin_8; //turn on PD8
			GPIOD->ODR &= ~(GPIO_Pin_9);
			GPIOB->ODR &= ~(GPIO_Pin_15);
			break;
		case Temp_ADC6:
			GPIOB->ODR |= GPIO_Pin_15; //turn on PB15
			GPIOD->ODR &= ~(GPIO_Pin_8 | GPIO_Pin_9);
			break;
		case Temp_ADC7:
			GPIOD->ODR &= ~(GPIO_Pin_8 | GPIO_Pin_9); //turn off PD8-PD9
			GPIOB->ODR &= ~(GPIO_Pin_15); //turn off PB15
			break;
		case Temp_ADC8:
			GPIOD->ODR |= GPIO_Pin_8; //turn on PD8
			GPIOB->ODR |= GPIO_Pin_15; //turn on PB15
			GPIOD->ODR &= ~(GPIO_Pin_9);
			break;
		case Water_ADC:
			break;
		default:
			break;
	}
}

extern void read_ADC(ADC_sensors_t ADC_sensor_x){
	Read_Position = ADC_sensor_x;
	ADC_count = 0;
	ADC_SoftwareStartConv(ADC1);
}

extern uint16_t return_ADC_value(ADC_sensors_t ADC_sensor_x){
	if(ADC_sensor_x <= Curr_ADC8){
		average= calculate_average(ADC_sensor_x)*1.79;
		return double_to_int(average);
	} else if((ADC_sensor_x >= Temp_ADC1) && (ADC_sensor_x <= Temp_ADC8)){
		average= calculate_average(ADC_sensor_x)*1.79;
		return double_to_int(average);
	} else if(ADC_sensor_x == Water_ADC){
		average = calculate_average(ADC_sensor_x)*0.088;
		return double_to_int(average);
	} else return 0;
}

extern void ADCTask(void *dummy){
	while(1){
		set_motor_current_temp_MUX(ADC_sensor);
		set_ADC_channel(ADC_sensor);
		vTaskDelay(1);
		read_ADC(ADC_sensor);
		ADC_sensor++;
		if(ADC_sensor > Water_ADC) ADC_sensor = Curr_ADC1;
		vTaskDelay(100);
	}
}

void ADC_IRQHandler(void){
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	ADC_Values[Read_Position][ADC_count] = ADC1->DR;
	if(ADC_count < (AMMOUNT_OF_RECORDED_VALUES - 1)){
		ADC_count++;
		ADC_SoftwareStartConv(ADC1);
	}
}
