/*
 * FSM.c
 *
 *  Created on: Feb 3, 2017
 *      Author: asus-andy
 */
#include "FSM.h"
#include "Buffer.h"
#include "motors.h"
#include "FreeRTOS.h"
#include "task.h"
#include "simple_UART.h"

#define CHAR_TO_INT (48)

extern void FSM(void *dummy){
	//initialize the FSM and UART
	UART_init();

	//inputBuffer.size = 0;

	//temporary storage to return from buffer
	char commandString[MAX_BUFFER_SIZE] = "";
	int tempVar;

	while(1){
		//it's important that this is while, if the task is accidentally awaken it
		//can't execute without having at least one item the input puffer
		while(inputBuffer.size == 0){

			//sleeps the task into it is notified to continue
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		}
		//Write a statement here to do a string comparison on commands
		Buffer_pop(&inputBuffer, commandString);

		//UART_push_out_len('\0', 1);


		UART_push_out("commandString: ");
		UART_push_out(commandString);
		UART_push_out("\n");


		char argument = commandString[3];
		commandString[3] = '\0';

		char tempOutputString[MAX_BUFFER_SIZE] = "";


		// MxF command
		if(commandString[0] == 'M' && commandString[2] == 'F'){


			GPIOD->ODR ^= GPIO_Pin_12;

			//int motor = commandString[1] - CHAR_TO_INT - 1; // zero based
			//Motor_Speed(motor, ((unsigned int)(argument)), Forward);
		}
		/*
		// MxR command
		else if(commandString[0] == 'M' && commandString[2] == 'R'){
			int motor = commandString[1] - CHAR_TO_INT - 1; // zero based
			Motor_Speed(motor, ((unsigned int)(argument)), Reverse);
		}
		// PWx command
		else if(strncmp(commandString, "PW", 2) == 0){
			int motor = commandString[2] - CHAR_TO_INT - 1; // zero based
			Motor_PWM(motor, ((unsigned int)(argument)* (10000 / 255)));
		}
		// RVx command
		else if(strncmp(commandString, "RV", 2) == 0){
			// Check if all motors are selected
			if(commandString[2] == 'A')
			{
				// Find rev/s for motor1
				tempVar = read_frequency_rpm(motor1) / CYCLES_PER_REV;
				tempOutputString[0] = '(';
				tempOutputString[1] = (char)(tempVar & 0xFF);
				tempOutputString[2] = (char)((tempVar >> 8) & 0xFF);
				tempOutputString[3] = ')';
				UART_push_out_len(tempOutputString, 4);

				// Find rev/s for motor2
				tempVar = read_frequency_rpm(motor2) / CYCLES_PER_REV;
				tempOutputString[0] = '(';
				tempOutputString[1] = (char)(tempVar & 0xFF);
				tempOutputString[2] = (char)((tempVar >> 8) & 0xFF);
				tempOutputString[3] = ')';
				UART_push_out_len(tempOutputString, 4);

				// Find rev/s for motor3
				tempVar = read_frequency_rpm(motor3) / CYCLES_PER_REV;
				tempOutputString[0] = '(';
				tempOutputString[1] = (char)(tempVar & 0xFF);
				tempOutputString[2] = (char)((tempVar >> 8) & 0xFF);
				tempOutputString[3] = ')';
				UART_push_out_len(tempOutputString, 4);
			}
			else{
				// Convert char to int to get the required motor ('0' is 48 in ASCII)
				int motor = commandString[2] - CHAR_TO_INT - 1; // zero based

				// Determine the rev/s
				tempVar = read_frequency_rpm(motor) / CYCLES_PER_REV;
				tempOutputString[0] = (char)(tempVar & 0xFF);
				tempOutputString[1] = (char)((tempVar >> 8) & 0xFF);
				UART_push_out_len(tempOutputString, 2);
			}

			// For RVx command, we always reach here
			// Setup end of command
			tempOutputString[0] = '\r';
			tempOutputString[1] = '\n';
			tempOutputString[2] = '\0';
			UART_push_out(tempOutputString);
		}
		// SMx command
		else if(strncmp(commandString, "SM", 2) == 0){
			int motor = commandString[2] - CHAR_TO_INT - 1; // zero based
			if(motor >= motor1 && motor <= motor3){
				// Stop motor
				Motor_Speed(motor, 0, Forward);

				// Send ACK
				strcpy(tempOutputString, "ACK\r\n");
				UART_push_out(tempOutputString);
			}
		}
		else if(strcmp(commandString, "STP") == 0){
			Motors_Stop();

			// Send ACK
			strcpy(tempOutputString, "ACK\r\n");
			UART_push_out(tempOutputString);
		}
		//RID
		else if(strcmp(commandString, "RID") == 0){
			UART_push_out("Motor ");
			UART_push_out("Control");
			UART_push_out("ler\r\n");
		}
		//Get Temperature
		else if(strcmp(commandString, "TMP") == 0){
			tempVar = actual_temperature;
			tempOutputString[0] = (char)(tempVar /10 + 48);
			tempOutputString[1] = (char)(tempVar % 10 + 48);
			tempOutputString[2] = '\r';
			tempOutputString[3] = '\n';
			UART_push_out_len(tempOutputString, 4);
			}
		//catch all error
		*/




	}
}

void FSM_Init(){

	// Create the FSM task
    xTaskCreate(FSM,
		(const signed char *)"FSM",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */
}
