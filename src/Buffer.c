/************************************************************************
  * @file    Buffer.c
  * @author  Andy Bates, Poorna Kalindas, Robert Keen, Gabriel Green
  * @version V1.0.0
  * @date    1-August-2019
  * @brief   This file provides a circular FIFO Buffer.
  * 
  *  @verbatim
  *  @endverbatim
  */
 
 /* Includes ----------------------------------------------------------*/
#include "Buffer.h"
#include <stdlib.h>

/**
  * @user	None
  * @brief  Adds an element to the end of the queue
  *				+ str (the element) must be NULL terminated ('\0') for strcpy
  * @param  b: pass a buffer by reference
  *				+ str: the element to be copied into the buffer
  */
extern void Buffer_add(Buffer_t* b, const char* str, uint8_t len){
	// Insert element
	memcpy(b->data[b->idx_to_load], str, len);
	b->data_len[b->idx_to_load] = len;
	b->idx_to_load++;
	b->idx_to_load %= MAX_BUFFER_SIZE;
	b->size++;

	// Check if buffer is full
	if(b->size > MAX_BUFFER_SIZE)
	{
		// Remove oldest element
		b->idx_to_pop++;
		b->idx_to_pop %= MAX_BUFFER_SIZE;
		b->size--;
		b->overflow_cnt++;
	}
}

/**
  * @user	None
  * @brief  Removes an element from the front of the queue
  * @param  b: pass a buffer by reference
  *				+ data: char variable by reference
  * @retval data gets element
  */
extern int Buffer_pop(Buffer_t* b, char* data){
	uint8_t ret = 0;

	// Check if the buffer has anything to pop
	if(b->size)
	{
		// Pop oldest element and store it in data
		memcpy(data, b->data[b->idx_to_pop], b->data_len[b->idx_to_pop]);
		ret = b->data_len[b->idx_to_pop];
		b->idx_to_pop++;
		b->idx_to_pop %= MAX_BUFFER_SIZE;
		b->size--;
	}

	return ret;
}

/**
  * @user	None
  * @brief  Reset all variables of the buffer
  * @param  b: pass a buffer by reference
  */
extern void Buffer_init(Buffer_t* b){
	b->idx_to_load = 0;
	b->idx_to_pop = 0;
	b->size = 0;
	b->overflow_cnt = 0;
}

/**
  * @user	None
  * @brief  Get the size of the buffer
  * @param  b: pass a buffer by reference
  * @retval buffer size
  */
extern int Buffer_size(Buffer_t* b){
	return b->size;
}

/**
  * @user	None
  * @brief  Get the number of overflows that have occurred
  * @param  b: pass a buffer by reference
  * @retval overflow counter
  */
extern int Buffer_overflow(Buffer_t* b){
	return b->overflow_cnt;
}
