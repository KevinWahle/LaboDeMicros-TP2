/***************************************************************************//**
  @file     circular_buffer.c
  @brief    Funciones para manejo del buffer circular
  @author   Grupo 5
 ******************************************************************************/

#include "circular_buffer.h"
#include <stdint.h>
#include <stdbool.h>

static uint8_t getCircularPointer(uint8_t index){
	return index % BUFFER_SIZE;
}

void CBinit(circularBuffer * CB){
	CB->head = 0;
	CB->tail = 0;
}

bool CBisEmpty(circularBuffer * CB){
	return CB->head == CB->tail;
}

void CBputChain(circularBuffer * CB, void * data, uint8_t bytesLen){
	for (uint8_t i = 0; i < bytesLen; ++i) {
		CB->buffer[CB->head] = ((uint8_t*)data)[i];
		CB->head = getCircularPointer(++CB->head);
	}
}

void CBputByte(circularBuffer * CB, uint8_t by){
	CB->buffer[CB->head] = by;
	CB->head = getCircularPointer(++CB->head);
}

uint8_t CBgetByte(circularBuffer * CB){
	if(CB->head != CB->tail){
		uint8_t data = CB->buffer[CB->tail];
		CB->tail = getCircularPointer(++CB->tail);
		return data;
	}
	return BUFFER_EMPTY;
}

uint8_t CBgetBufferState(circularBuffer * CB){
	if(CB->head >= CB->tail)
		return CB->head - CB->tail;
	else
		return BUFFER_SIZE - CB->tail + CB->head;
}
/*
const uint8_t * CBgetData(circularBuffer * CB, uint8_t bytesLen){
}
*/
void CBreset(circularBuffer * CB){
	for (uint8_t i= 0; i < BUFFER_SIZE; ++i){
		CB->buffer[i] = 0;
	}
	CB->tail = 0;
	CB->head = 0;
}

