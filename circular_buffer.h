/***************************************************************************//**
  @file     circular_buffer.h
  @brief    Funciones para manejo del buffer circular
  @author   Grupo 5
 ******************************************************************************/


#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#define BUFFER_SIZE 12  // 12 bytes buffer size JUST 11 bytes can be used for store data without deleting older bytes
#define BUFFER_EMPTY 255 // Special number used for notify that the buffer is empty

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct{
	uint8_t buffer[BUFFER_SIZE];

	//private
	uint8_t head;
	uint8_t tail;
}circularBuffer;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
/**
 * @brief builder
 * @param circularBuffer type
 */
void CBinit(circularBuffer * CB);
/**
 * @brief tells you whether there is new data
 * @param circularBuffer type
 */
bool CBisEmpty(circularBuffer * CB);
/**
 * @brief push a string into the buffer
 * @param circularBuffer type, data, data length in bytes
 */
void CBputChain(circularBuffer * CB, void * data, uint8_t bytesLen);
/**
 * @brief push a byte into the buffer
 * @param circularBuffer type, data
 */
void CBputChar(circularBuffer * CB, uint8_t ch);
/**
 * @brief gets a byte from the buffer
 * @param circularBuffer type
 * @return the byte or BUFFER_FULL special byte
 */
uint8_t CBgetChar(circularBuffer * CB);
/**
 * @brief tells you the amount of unread bytes
 * @param circularBuffer type
 */
uint8_t CBgetBufferState(circularBuffer * CB);

/** UNIMPLEMENTED
 * @brief gets a chain of bytes
 * @param
 */

//const uint8_t *  CBgetData(circularBuffer * CB, uint8_t bytesLen);

/**
 * @brief Set all bytes to 0
 * @param circularBuffer type
 */
void CBreset(circularBuffer * CB);

/*******************************************************************************
 ******************************************************************************/

#endif // _CIRCULAR_BUFFER_H_






