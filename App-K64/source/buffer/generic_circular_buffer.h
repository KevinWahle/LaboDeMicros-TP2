/***************************************************************************//**
  @file     generic_circular_buffer.h
  @brief    Funciones para manejo del buffer circular
  @author   Grupo 5
 ******************************************************************************/


#ifndef _GENERIC_CIRCULAR_BUFFER_H_
#define _GENERIC_CIRCULAR_BUFFER_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#define BUFFER_BYTES_SIZE 256 // real size can be calculated as  BUFFER_BYTES_SIZE/sizeof(DATA_TYPE)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct{

	//private
	uint8_t sizeDataType;
	uint8_t buffer[BUFFER_BYTES_SIZE];
	uint8_t head;
	uint8_t tail;
} genericCircularBuffer;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief builder
 * @param circularBuffer type
 */
void GCBinit(genericCircularBuffer * CB, uint8_t sizeDataType);

/**
 * @brief tells you whether there is new data
 * @param circularBuffer type
 */
bool GCBisEmpty(genericCircularBuffer * CB);

/**
 * @brief push a string into the buffer
 * @param circularBuffer type, data, data length in bytes
 */
void GCBputDataChain(genericCircularBuffer * CB, const void * dataChain, uint8_t AmountOfData);

/**
 * @brief push a byte into the buffer
 * @param circularBuffer type, data
 */
void GCBputData(genericCircularBuffer * CB, void* dataToPush);

/**
 * @brief gets a byte from the buffer
 * @param circularBuffer type
 * @return the byte or BUFFER_FULL special byte
 */
void GCBgetData(genericCircularBuffer * CB, void* dataReturn);

/**
 * @brief tells you the amount of unread bytes
 * @param circularBuffer type
 */
uint8_t GCBgetBufferState(genericCircularBuffer * CB);

/** UNIMPLEMENTED
 * @brief gets a chain of bytes
 * @param
 */

//const uint8_t *  CBgetData(circularBuffer * CB, uint8_t bytesLen);

/**
 * @brief Set all bytes to 0
 * @param circularBuffer type
 */
void GCBreset(genericCircularBuffer * CB);

/*******************************************************************************
 ******************************************************************************/

#endif // _CIRCULAR_BUFFER_H_

