/***************************************************************************//**
  @file     circular_buffer.h
  @brief    Funciones para manejo del buffer circular
  @author   Grupo 5
 ******************************************************************************/


#ifndef _SPI_BUFFER_H_
#define _SPI_BUFFER_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#define SPIBUFFER_SIZE 32  // 12 bytes buffer size JUST 11 bytes can be used for store data without deleting older bytes

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct{
	package buffer[SPIBUFFER_SIZE];

	//private
	uint8_t head;
	uint8_t tail;
} SPIBuffer;

typedef struct{
	uint8_t msg;
	uint8_t* pSave;		//Puntero donde se guarda lo leído
	uint8_t read	:1;
	//Agregar start y end de la transmision para gestionar PCs
} package;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief builder
 * @param SPIBuffer type
 */
void CBinit(SPIBuffer * CB);

/**
 * @brief tells you whether there is new data
 * @param SPIBuffer type
 */
bool CBisEmpty(SPIBuffer * CB);

/**
 * @brief push a string into the buffer
 * @param SPIBuffer type, data, data length in bytes
 */
void CBputChain(SPIBuffer * CB, package *data, uint8_t bytesLen);

/**
 * @brief push a byte into the buffer
 * @param SPIBuffer type, data
 */
void CBputByte(SPIBuffer * CB, uint8_t by);

/**
 * @brief gets a byte from the buffer
 * @param circularBuffer type
 * @return the byte or BUFFER_FULL special byte
 */
package CBgetPckg(SPIBuffer * CB);

/**
 * @brief tells you the amount of unread bytes
 * @param SPIBuffer type
 */
uint8_t CBgetBufferState(SPIBuffer * CB);

/** UNIMPLEMENTED
 * @brief gets a chain of bytes
 * @param
 */

//const uint8_t *  CBgetData(SPIBuffer * CB, uint8_t bytesLen);

/**
 * @brief Set all bytes to 0
 * @param circularBuffer type
 */
void CBreset(SPIBuffer * CB);

/*******************************************************************************
 ******************************************************************************/

#endif // _SPI_BUFFER_H_






