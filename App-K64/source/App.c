/***************************************************************************//**
  @file     App.c
  @brief    TP2: Comunicacion Serie
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "AccMagSensor/AccMagSensor.h"
#include "CAN/CAN.h"
#include "timer/timer.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define UART_ID		0

#define ANGLE_DELTA	5	// °

#define TIME_SENS_CHECK	50	//ms

#define MIN_SEND_CYCLES	20	// Ciclos maximos que pueden pasar sin mandar inclinacion

#define ANGLE_ID_ARR	{ 'R', 'C', 'O' }

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

void checkSensorAndSend();

/*******************************************************************************
 *******************************************************************************
                        GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/

tim_id_t timer;

uint32_t values[3];		// Guarda los ultimos valores del sensor

const char angleID[] = ANGLE_ID_ARR;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{

	CANInit();

	timerInit();

	timer = timerGetId();

	timerStart(timer, TIMER_MS2TICKS(TIME_SENS_CHECK), TIM_MODE_PERIODIC, NULL);

}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	FXOS8700CQ_init();
	while (1) {
		if (timerExpired(timer)) {
			checkSensorAndSend();
		}
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void checkSensorAndSend() {

	static uint32_t noSend[3];

	uint32_t prevValues[3];

	// readValues(values);

	for (int i = 0; i < 3; i++) {

		if (values[i] >= prevValues[i] + ANGLE_DELTA || values[i] <= prevValues[i] - ANGLE_DELTA || noSend[i] > MIN_SEND_CYCLES) {		// El eje se movio, o paso
																						// tiempo sin mandar



			//

			noSend[i] = 0;
		}
		else {		// La placa no se movio
			noSend++;
		}

	}



}

/*******************************************************************************
 ******************************************************************************/
