/*
 * I2C.c
 *
 *  Created on: Jun 4, 2018
 *      Author: robert
 */
#include "stm32f4xx.h"
#include "I2C.h"
#include "string.h"
#include "I2C_Sensors.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "simple_UART.h"
#include "stdlib.h"
#include "string.h"


#define I2C_SADD_BIT	1

#define I2C_READ_BIT 0x01

//I2C Output buffer
#define I2C_OUTPUT_BUFFER_SIZE	8
uint8_t I2C_OutputBuffer[I2C_OUTPUT_BUFFER_SIZE];

typedef enum{
	read,
	write
} I2C_state_t;

//I2C Input Buffer
uint8_t *I2C_inputBuffer;

//bytes count
volatile uint8_t bytes_count = 0;

//bytes total
volatile uint8_t bytes_total;

//slave address
uint8_t slave_address;

//I2C state
I2C_state_t I2C_state;

//FreeRTOS current task handle
TaskHandle_t TaskToNotify = NULL;



extern uint16_t switch_endiness_uint16(uint16_t input) {
	uint8_t temp = (input & 0xFF00) >> 8;
	input = (input & 0x00FF) << 8;
	input |= temp;
	return input;
}

extern uint32_t switch_endiness_uint32(uint32_t input, uint8_t numBytes) {

	uint32_t output = 0;

	for(uint8_t i = 0; i < numBytes; i++) {
		output = output << 8;
		output |= (input & 0xFF);
		input = input >> 8;
	}

	return output;
}

extern void I2C_setup(){

	//Enable clock for GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//GPIOB setup for SCL and SDA
	GPIO_InitTypeDef GPIOB_Init;
	GPIOB_Init.GPIO_Mode = GPIO_Mode_AF;
	GPIOB_Init.GPIO_OType = GPIO_OType_OD;
	GPIOB_Init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIOB_Init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOB_Init.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIOB_Init);

	//sets alternate function register to AF4 for PB7 and PB6
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	//NVIC Setup
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);

	/* Configure IT */
	/* (3) Set priority for USART1_IRQn */
	/* (4) Enable USART1_IRQn */
	NVIC_SetPriority(I2C1_EV_IRQn, 7); /* (3) */
	NVIC_EnableIRQ(I2C1_EV_IRQn); /* (4) */

}

static void I2C_init(){
	//Enable clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	//I2C setup
	I2C_InitTypeDef I2C_Init_Struct;
	I2C_Init_Struct.I2C_Ack = I2C_Ack_Enable;
	I2C_Init_Struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init_Struct.I2C_Mode = I2C_Mode_I2C;
	I2C_Init_Struct.I2C_OwnAddress1 = 0x0;
	I2C_Init_Struct.I2C_ClockSpeed = 50000;
	I2C_Init_Struct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_Init(I2C1, &I2C_Init_Struct);

	//Enable I2C
	I2C1->CR1 |= I2C_CR1_PE;

	//Enable interrupts
	I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
}

extern void I2C_read(uint8_t address, uint8_t numBytes, uint8_t *message){
	TaskToNotify = xTaskGetCurrentTaskHandle();
	I2C_init();
	slave_address = address;
	I2C_state = read;
	bytes_total = numBytes;
	I2C_inputBuffer = message;
	I2C1->CR1 |= I2C_CR1_START;
}

extern void I2C_write(uint8_t address, uint8_t numBytes, uint8_t message[]){
	TaskToNotify = xTaskGetCurrentTaskHandle();
	I2C_init();
	I2C_state = write;
	slave_address = address;
	bytes_total = numBytes;
	memcpy(I2C_OutputBuffer, message, numBytes);
	I2C1->CR1 |= I2C_CR1_START;
}

void I2C1_EV_IRQHandler(void) {
	//Waits for start bit
	if((I2C1->SR1 & I2C_SR1_SB) == I2C_SR1_SB){
		if(I2C_state == write){
			I2C1->DR |= (slave_address << I2C_SADD_BIT);
			I2C1->DR &= ~(I2C_READ_BIT);
		} else if(I2C_state == read){
			I2C1->DR |= (slave_address << I2C_SADD_BIT) | I2C_READ_BIT;
			if( bytes_total == 2 ){
				I2C1->CR1 |= I2C_CR1_POS;
				I2C1->CR1 &= ~(I2C_CR1_ACK);
			}else if( bytes_total == 1 ){
				I2C1->CR1 &= ~(I2C_CR1_ACK);
			}
			I2C1->SR2;
		}
	//Waits for address sent bit
	}else if((I2C1->SR1 & I2C_SR1_ADDR) == I2C_SR1_ADDR){
		if(I2C_state == write){
			//needs to read SR2 to clear ADDR bit and continue
			I2C1->SR2;
			I2C1->DR = I2C_OutputBuffer[bytes_count];
			bytes_count++;
			bytes_total--;
		} else if(I2C_state == read){
			//the following is the read process specified by the F4 reference
			//manual page 482-483
			if( bytes_total == 1 ){
				I2C1->CR1 &= ~(I2C_CR1_ACK);
			}
			I2C1->SR2;
		}
	//Writes data to I2C
	}else if( ((I2C1->SR1 & I2C_SR1_TXE) == I2C_SR1_TXE) && ((I2C1->SR1 & I2C_SR1_BTF) == I2C_SR1_BTF)){
		if(bytes_total >= 1){
			I2C1->DR = I2C_OutputBuffer[bytes_count];
			bytes_total--;
			bytes_count++;
		}else if(bytes_total == 0){
			bytes_count = 0;
			I2C1->CR1 |= I2C_CR1_STOP;
			I2C1->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
			I2C_Cmd(I2C1, DISABLE);
			I2C_DeInit(I2C1);
			vTaskNotifyGiveFromISR(TaskToNotify, pdFALSE);
		}
	//Reads I2C data
	}else if( ((I2C1->SR1 & I2C_SR1_RXNE) == I2C_SR1_RXNE) && ((I2C1->SR1 & I2C_SR1_BTF) == I2C_SR1_BTF) ){
		if(bytes_total > 2){
			*I2C_inputBuffer = I2C1->DR;
			I2C_inputBuffer++;
			bytes_total--;
			if(bytes_total == 2){
				I2C1->CR1 &= ~(I2C_CR1_ACK);
				I2C1->CR1 |= I2C_CR1_POS;
			}
		} else if(bytes_total == 2){
			*I2C_inputBuffer = I2C1->DR;
			I2C1->CR1 |= I2C_CR1_STOP;
			I2C_inputBuffer++;
			bytes_total--;
			*I2C_inputBuffer = I2C1->DR;
			I2C1->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
			I2C1->CR1 |= I2C_CR1_ACK;
			I2C_Cmd(I2C1, DISABLE);
			I2C_DeInit(I2C1);
			vTaskNotifyGiveFromISR(TaskToNotify, pdFALSE);
		} else if(bytes_total == 1){
			I2C1->CR1 |= I2C_CR1_STOP;
			*I2C_inputBuffer = I2C1->DR;
			I2C1->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
			I2C1->CR1 |= I2C_CR1_ACK;
			I2C_Cmd(I2C1, DISABLE);
			I2C_DeInit(I2C1);
			vTaskNotifyGiveFromISR(TaskToNotify, pdFALSE);
		}
	//Runs in the case of failure
	} else if((I2C1->SR1 & I2C_SR1_AF) == I2C_SR1_AF){
		bytes_count = 0;
		I2C1->CR1 |= I2C_CR1_STOP;
		I2C1->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
		I2C_Cmd(I2C1, DISABLE);
		I2C_DeInit(I2C1);
		vTaskNotifyGiveFromISR(TaskToNotify, pdFALSE);
	}
}


