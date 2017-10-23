/*
 * simple_UART.c
 *
 *  Created on: Aug 19, 2017
 *      Author: abates
 */

#include "simple_UART.h"
//#include "buffer.h"
//#include "FSM.h"
//#include "LEDs.h"
#include "FreeRTOS.h"
#include "Task.h"
#include <string.h>

char stringtosend[MAX_OUPUT_DATA] = "";
char UARTInput[MAX_BUFFER_DATA];
uint8_t UARTInputIndex = 0;

TaskHandle_t UARTTaskToNotify = NULL;

//uint8_t bytes_to_send;

static void Configure_GPIO_USART1(void) {
	/* Enable the peripheral clock of GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Configuration: TIM5 CH1 (PA0) */
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect USART1 pins to AF */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
}

/**
 * @brief  This function configures USART1.
 * @param  None
 * @retval None
 */
static void Configure_USART1(void) {
	/* Enable the peripheral clock USART1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//RCC->CFGR3 |= RCC_CFGR3_USART1SW_1;
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization

	USART_InitStruct.USART_BaudRate = 9600;	// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;	// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);

	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	USART1->CR1 |= 0b100000; //Enable the USART1 receive interrupt


	/* Configure IT */
	/* (3) Set priority for USART1_IRQn */
	/* (4) Enable USART1_IRQn */
	NVIC_SetPriority(USART1_IRQn, 7); /* (3) */
	NVIC_EnableIRQ(USART1_IRQn); /* (4) */

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

extern void UART_push_out(char* mesg) {

	for (int i = 0; i < strlen(mesg); i++) {

		//We need to wait for data register to be empty
		while (!(USART1->SR & 0x00000040)); //Checks if the TC bit in the USART_SR register is high

		USART1->DR = mesg[i];
	}
}

extern void UART_push_out_len(char* mesg, int len) {

	for (int i = 0; i < len; i++) {

		//We need to wait for data register to be empty
		while (!(USART1->SR & 0x00000040)); //Checks if the TC bit in the USART_SR register is high

		USART1->DR = mesg[i];
	}
}

extern void UART_init() {

	//initialize the input buffer
	Buffer_init(&inputBuffer);

	//Get current task handle
	UARTTaskToNotify = xTaskGetCurrentTaskHandle();

	//initialize the UART driver
	Configure_GPIO_USART1();
	Configure_USART1();
}

void USART1_IRQHandler() {
	char tempInput[1];
	tempInput[0] = USART1->DR;

	//Check for new line character which indicates end of command
	if (tempInput[0] == '\n') {
		Buffer_add(&inputBuffer, UARTInput, MAX_BUFFER_DATA);
		memset(UARTInput, 0, 8);
		UARTInputIndex = 0;

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(UARTTaskToNotify, &xHigherPriorityTaskWoken);

	} else {
		UARTInput[UARTInputIndex] = tempInput[0];
		UARTInputIndex = (UARTInputIndex + 1) & 7;
	}
}
