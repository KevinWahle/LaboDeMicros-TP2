/***************************************************************************//**
  @file     FXOS8700CQ.h
  @brief    FXOS8700CQ Accelerometer and magnetometer
  @author   Grupo 5
  @date		30 sep. 2022
 ******************************************************************************/

#ifndef _FXOS8700CQ_H_
#define _FXOS8700CQ_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "I2Cm/I2Cm.h"
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}ACCEL;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}MAG;
/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: extern unsigned int anio_actual;+


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
/**
 * @brief configures the sensor. Blocking function til it initalizes the sensor
 */
void FXOS8700CQ_init(void);
/**
 * @brief Async. Start the reading of Acc and Mag
 */
void startReadAccMagFXOS8700CQ(void);
/**
 * @brief gets last data read
 * @param ACCEL and MAG structure where data 'll be placed
 */
void getLastReadFXOS8700CQ(ACCEL * acc, MAG * mg);

/*******************************************************************************
 ******************************************************************************/

#endif // _FXOS8700CQ_H_
