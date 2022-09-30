/***************************************************************************//**
  @file     App.c
  @brief    TP2: Comunicacion Serie
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

//#include "I2Cm/I2Cm.h"
#include "timer/timer.h"
#include "MCAL/gpio.h"
#include "MCAL/board.h"
#include "AccMagSensor/AccMagSensor.h"
/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define I2C_ID		I2C_ACC
#define I2C_ADDR	0x1D


void App_Init (void)
{
	gpioMode(PIN_LED_RED, OUTPUT);
	gpioWrite(PIN_LED_RED, HIGH);
	timerInit();
}

void App_Run (void)
{
	init_ACC_MAG();
	ACCEL acc;
	MAG mg;
	while(1){
		startReading_ACC_MAG();
		timerDelay(TIMER_MS2TICKS(1000));
		gpioToggle(PIN_LED_RED);
		if(newDataAvailable_ACC_MAG()){
			getLastRead_ACC_MAG(&acc, &mg);
		}
	}
}
