/***************************************************************************//**
  @file     AccMagSensor.h
  @brief    AccMagSensor Lectura de sensor no bloqueante
  @author   Grupo 5
  @date		23 sep. 2022
 ******************************************************************************/

#ifndef _AccMagSensor_H_
#define _AccMagSensor_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
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
void init_ACC_MAG();

bool startReading_ACC_MAG(void);
bool newDataAvailable_ACC_MAG(void);
void getLastRead_ACC_MAG(ACCEL * acc, MAG * mg);

/*******************************************************************************
 ******************************************************************************/

#endif // _AccMagSensor_H_
