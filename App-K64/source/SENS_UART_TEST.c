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

#include "UART/uart.h"

#include <stdio.h>
#include <math.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define I2C_ID		I2C_ACC
#define I2C_ADDR	0x1D

#define UART_ID	0

const uart_cfg_t configUART = {.baudrate=115200, .parity=NO_PARITY, .MSBF=false};

const double gFactor = 488e-6;

tim_id_t timer;

size_t num2String(uint32_t num, char* str);

void App_Init (void)
{
	timerInit();
	gpioMode(PIN_LED_RED, OUTPUT);
	gpioWrite(PIN_LED_RED, HIGH);
	uartInit(UART_ID, configUART);

	timer = timerGetId();
	timerStart(timer, TIMER_MS2TICKS(50), TIM_MODE_PERIODIC, NULL);

	uartWriteMsg(UART_ID, "Accx\tAccY\tAccZ\tGyrX\tGyrY\tGyrZ\r\n", 31);
}

void App_Run (void)
{
	init_ACC_MAG();
	ACCEL acc;
	MAG mg;
	while(1){
		startReading_ACC_MAG();
		timerDelay(TIMER_MS2TICKS(200));
		gpioToggle(PIN_LED_RED);
		getLastRead_ACC_MAG(&acc, &mg);
//		if (timerExpired(timer) && uartIsTxMsgComplete(UART_ID)) {
			double Acc[3] = { acc.x*gFactor, acc.y*gFactor, acc.z*gFactor };

			double yaw = atan2(Acc[0], sqrt(pow(Acc[1],2) + pow(Acc[2],2)))*(180.0/3.14);
			double pitch = atan2(Acc[1], sqrt(pow(Acc[0],2) + pow(Acc[2],2)))*(180.0/3.14);
//			double roll = atan2(acc.z, sqrt(pow(acc.x,2) + pow(acc.y,2)))*(180.0/3.14);
			double roll = atan2(Acc[0], Acc[2])*(180.0/3.14);

			char msg[128];
			size_t size = sprintf(msg, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", yaw, pitch, roll, Acc[0], Acc[1], Acc[2]);
//			size_t size = sprintf(msg, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", 0.59, 0.2, 0.3, 0.4, 0.5, 0.666);
			uartWriteMsg(UART_ID, msg, size);
//		}
	}
}


