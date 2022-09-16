/***************************************************************************//**
  @file     I2C.h
  @brief    I2C Functions
  @author   Grupo 5
  @date		13 sep. 2022
 ******************************************************************************/

#ifndef _I2Cm_H_
#define _I2Cm_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/



/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint8_t I2CPort_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: extern unsigned int anio_actual;+


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Inicializa el modulo I2C
 * @param id: Instancia del I2C
*/
void I2CmInit(I2CPort_t id);

/**
 * @brief Inicializa el modulo I2C
 * @param param1 Descripcion parametro 1
 * @param param2 Descripcion parametro 2
 * @return Descripcion valor que devuelve
*/
void I2CmStartTransaction(uint8_t address, uint8_t* writeBuffer, uint8_t writeSize, uint8_t* readBuffer, uint8_t readSize);


/*******************************************************************************
 ******************************************************************************/

#endif // _I2Cm_H_
