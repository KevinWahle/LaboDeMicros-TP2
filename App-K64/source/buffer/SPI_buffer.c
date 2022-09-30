/***************************************************************************//**
  @file     circular_buffer.c
  @brief    Funciones para manejo del buffer circular
  @author   Grupo 5
 ******************************************************************************/

#include "SPI_buffer.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

package pckgNULL={.msg=0, .pSave=NULL, .read=0};

static uint8_t getCircularPointer(uint8_t index){
	return index % SPIBUFFER_SIZE;
}

void CBinit(SPIBuffer * CB){
	CB->head = 0;
	CB->tail = 0;
}

bool CBisEmpty(SPIBuffer * CB){
	return CB->head == CB->tail;
}

void CBputChain(SPIBuffer * CB, package *data, uint8_t Len){
	for (uint8_t i = 0; i < Len; ++i) {
		memcpy(&(CB->buffer[CB->head]), &(data[i]), sizeof(package));

		CB->head = getCircularPointer(++CB->head);
	}
}

void CBputByte(SPIBuffer * CB, package* pckg){
	memcpy( &(CB->buffer[CB->head]), &pckg, sizeof(package));

	CB->head = getCircularPointer(++CB->head);
}

package CBgetPckg(SPIBuffer * CB){
	if(CB->head != CB->tail){
		package data;
		memcpy(&data, &(CB->buffer[CB->tail]), sizeof(package));
		CB->tail = getCircularPointer(++CB->tail);
		return data;
	}
	return pckgNULL;
}

uint8_t CBgetBufferState(SPIBuffer * CB){
	if(CB->head >= CB->tail)
		return CB->head - CB->tail;
	else
		return SPIBUFFER_SIZE - CB->tail + CB->head;
}

void CBreset(SPIBuffer * CB){
	for (uint8_t i= 0; i < SPIBUFFER_SIZE; ++i){
		CB->buffer[i]=pckgNULL;
	}

	CB->tail = 0;
	CB->head = 0;
}

