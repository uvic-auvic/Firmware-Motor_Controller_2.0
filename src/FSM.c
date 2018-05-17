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
#include "motors.h"

#define NUMBER_OF_MOTORS	8

/** The following defines are determined using the input command structure.
 * 	Currently, this is stored in the Wiki section of the GitHub repository for this code
 */
// Motor Speed(MS) Commands (Motor forward and Motor reverse)
#define MS_MOTOR_NUMBER_LOCATION 0x01
#define MS_MOTOR_NUMBER_LENTGH	1
#define MS_ARGUMENT_LOCATION	0x03
#define MS_ARGUMENT_LENGTH		2
#define MS_COMMAND_LENGTH		5

// Pulse Width Modulation(PW) command
#define PW_MOTOR_NUMBER_LOCATION	0x02
#define PW_MOTOR_NUMBER_LENTGH		1
#define PW_ARGUMENT_LOCATION		0x03
#define PW_ARGUMENT_LENGTH			2
#define PW_COMMAND_LENGTH			5

// Stop Motor(SM) Command
#define SM_MOTOR_NUMBER_LOCATION	0x02
#define SM_MOTOR_NUMBER_LENTGH		1
#define SM_COMMAND_LENGTH			3

// Revolutions on motor(RV) Command
#define RV_MOTOR_NUMBER_LOCATION	0x02
#define RV_MOTOR_NUMBER_LENTGH		1
#define RV_COMMAND_LENGTH			3

// Calibrate Motor(CL) command
#define CL_MOTOR_NUMBER_LOCATION	0x02
#define CL_MOTOR_NUMBER_LENTGH		1
#define CL_COMMAND_LENGTH			3

// STP command
#define STP_COMMAND_LENGTH	3

// RID command
#define RID_COMMAND_LENGTH	3

// TMP command
#define TMP_COMMAND_LENGTH	3

//MSA command
#define	MSA_COMMAND_LENGTH	(3 + (NUMBER_OF_MOTORS * 3))


/* Global Variables */
//UART input commands buffer.
extern Buffer_t inputBuffer;

/** Converts ASCII to Integer for positive numbers and zero
 * Returns:
 * 	ASCII number as an Integer (0 - 99), or
 * 	-1 - Error: ASCII character is not a number
 */
int asciiToInt(char input[], uint8_t length) {

	int output = 0;

	for(int i = 0; i < length; i++) {

		if (input[i] >= '0' && input[i] <= '9') {
			output = output * 10;
			output += (int)(input[i] - '0');
		} else {
			return -1; //Error: ASCII character is not a number
		}
	}

	return output;
}

extern void FSM(void *dummy){

	//initialize UART
	UART_init();

	//temporary storage to return from buffer
	char commandString[MAX_BUFFER_DATA] = "";

	while(1){
		//it's important that this is while, if the task is accidentally awaken it
		//can't execute without having at least one item the input buffer
		while(inputBuffer.size == 0){

			//sleeps the task into it is notified to continue
			uint32_t ticks = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(5000) );
			//ticks is the amount of time the OS has left to wait before waking by time
			//if we wake by timeout for commands we want to disable all motors
			if(ticks == 0){
				stop_all_motors();
			}
		}

		//Pop command from buffer
		Buffer_pop(&inputBuffer, commandString);

		// MxF command
		if(commandString[0] == 'M' && commandString[2] == 'F' && strlen(commandString) == MS_COMMAND_LENGTH)	{

			int8_t motorNumber = asciiToInt(commandString + MS_MOTOR_NUMBER_LOCATION, MS_MOTOR_NUMBER_LENTGH);
			int8_t argument = asciiToInt(commandString + MS_ARGUMENT_LOCATION, MS_ARGUMENT_LENGTH);

			if (motorNumber < 1 || motorNumber > NUMBER_OF_MOTORS) {
				// send out error: "Invalid motor number"
				while(UART_push_out("ERR_MTR_IVD\r\n") == -2);

			} else if (argument < 0) {
				// send out error: "Argument must be a number between 00-99"
				while(UART_push_out("ERR_ARG_IVD\r\n") == -2);

			} else {
				motor_set_speed_percent(motorNumber, argument, Forward);
			}
		}

		// MxR command
		else if(commandString[0] == 'M' && commandString[2] == 'R' && strlen(commandString) == MS_COMMAND_LENGTH){

			int8_t motorNumber = asciiToInt(commandString + MS_MOTOR_NUMBER_LOCATION, MS_MOTOR_NUMBER_LENTGH);
			int8_t argument = asciiToInt(commandString + MS_ARGUMENT_LOCATION, MS_ARGUMENT_LENGTH);

			if (motorNumber < 1 || motorNumber > NUMBER_OF_MOTORS) {
				// send out error: "Invalid motor number"
				while(UART_push_out("ERR_MTR_IVD\r\n") == -2);

			} else if (argument < 0) {
				// send out error: "Argument must be a number between 00-99"
				while(UART_push_out("ERR_ARG_IVD\r\n") == -2);

			} else {
				motor_set_speed_percent(motorNumber, argument, Reverse);

			}
		}

		// PWx command
		else if(strncmp(commandString, "PW", 2) == 0 && strlen(commandString) == PW_COMMAND_LENGTH){

			int8_t motorNumber = asciiToInt(commandString + PW_MOTOR_NUMBER_LOCATION, PW_MOTOR_NUMBER_LENTGH);
			int8_t argument = asciiToInt(commandString + PW_ARGUMENT_LOCATION, PW_ARGUMENT_LENGTH);

			if (motorNumber < 1 || motorNumber > NUMBER_OF_MOTORS) {
				// send out error: "Invalid motor number"
				while(UART_push_out("ERR_MTR_IVD\r\n") == -2);

			} else if (argument < 0) {
				// send out error: "Argument must be a number between 00-99"
				while(UART_push_out("ERR_ARG_IVD\r\n") == -2);

			} else {
				//Motor_PWM(motorNumber, (argument)* (10000 / 255));

			}
		}

		// RVx command
		else if(strncmp(commandString, "RV", 2) == 0 && strlen(commandString) == RV_COMMAND_LENGTH){

			int8_t motorNumber = asciiToInt(commandString + RV_MOTOR_NUMBER_LOCATION, RV_MOTOR_NUMBER_LENTGH);

			if(commandString[RV_MOTOR_NUMBER_LOCATION] == 'A') {
				// Send out all motor revolutions
				for(uint8_t motor = 1; motor <= NUMBER_OF_MOTORS; motor++){
					uint16_t rpm = motor_get_rpm(motor);
					while(UART_push_out_len((char *)&rpm, 2) == -2);
				}
				while(UART_push_out_len("\r\n", 2) == -2);

			} else if (motorNumber < 1 || motorNumber > NUMBER_OF_MOTORS) {
				// send out error: "Invalid motor number"
				while(UART_push_out("ERR_MTR_IVD\r\n") == -2);

			} else {
				char bufs[5];
				uint16_t rpm = motor_get_rpm(motorNumber);
				itoa(rpm, bufs, 10);
				UART_push_out_len(bufs, 5);
				UART_push_out_len("\r\n", 2);
			}
		}

		// CLx command
		else if (strncmp(commandString, "CL", 2) == 0 && strlen(commandString) == CL_COMMAND_LENGTH) {

			int8_t motorNumber = asciiToInt(commandString + CL_MOTOR_NUMBER_LOCATION, CL_MOTOR_NUMBER_LENTGH);

			if (motorNumber < 1 || motorNumber > NUMBER_OF_MOTORS) {
				// send out error: "Invalid motor number"
				while(UART_push_out("ERR_MTR_IVD\r\n") == -2);

			} else {
				// Calibrate Motors
				// if (motor are calibrated successfully) {
				//		while(UART_push_out("GOOD\r\n") == -2);
				// }

			}
		}

		// SMx command
		else if(strncmp(commandString, "SM", 2) == 0 && strlen(commandString) == SM_COMMAND_LENGTH){

			int8_t motorNumber = asciiToInt(commandString + SM_MOTOR_NUMBER_LOCATION, SM_MOTOR_NUMBER_LENTGH);

			if (motorNumber < 1 || motorNumber > NUMBER_OF_MOTORS) {
				// send out error: "Invalid motor number"
				while(UART_push_out("ERR_MTR_IVD\r\n") == -2);

			} else {
				motor_set_speed_percent(motorNumber, 0, Forward);
				while(UART_push_out("ACK\r\n") == -2);
			}
		}

		// STP Command
		else if(strcmp(commandString, "STP") == 0){

			stop_all_motors();
			while(UART_push_out("ACK\r\n") == -2);
		}

		//RID command
		else if(strncmp(commandString, "RID", 3) == 0){

			while(UART_push_out("Motor Controller\r\n") == -2);

		}

		//TMP command
		else if(strcmp(commandString, "TMP") == 0){

			// Get motor temperature
			// Send out temperature

		}

		//MSA Command
		else if (strncmp(commandString, "MSA", 3) == 0 && strlen(commandString) == MSA_COMMAND_LENGTH){

			//The enumeration for motors starts at index 1
			for(uint8_t motor = 1; motor <= NUMBER_OF_MOTORS; motor++){
				char direction_idx = motor * 3;
				uint8_t speed_idx = direction_idx + 1;
				uint8_t speed = asciiToInt(&commandString[speed_idx], 2);
				if(commandString[direction_idx] == 'F'){
					motor_set_speed_percent(motor, speed, Forward);
				}else if (commandString[direction_idx] == 'R'){
					motor_set_speed_percent(motor, speed, Reverse);
				}else{
					//If we don't see R or F let's do nothing
				}
			}

		}

		// No matches
		else {
			// send out error: "Invalid command"
			while(UART_push_out("ERR_CMD_IVD\r\n") == -2);
			while(UART_push_out(commandString) == -2);
			while(UART_push_out("\r\n") == -2);
		}
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
