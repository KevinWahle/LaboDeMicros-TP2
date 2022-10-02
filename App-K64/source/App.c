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
#include "UART/uart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define PI 3.14159265359

#define UART_ID		0

#define ANGLE_DELTA	5	// °

#define TIME_SENS_CHECK	50	//ms

#define TIME_UART_SEND	100	//ms

#define MIN_SEND_CYCLES	20	// Ciclos maximos que pueden pasar sin mandar inclinacion

#define MAX_UART_MSG	32

#define GROUPS_COUNT	8

#define GROUP_NUMBER	5

#define CAN_BASE_ID		0x100

#define CAN_ID			(CAN_BASE_ID + GROUP_NUMBER)

#define AXIS_COUNT		3

#define ANGLE_ID_ARR	{ 'R', 'C', 'O' }

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef int16_t angle_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void checkSensorAndSend();

// Funcion donde se guarda los valores recibidos al array
static void parseCANMsg(angle_t angles[AXIS_COUNT], char* msg, size_t length);

static void sendAngleUART();

void readAngles(angle_t anglesArr[AXIS_COUNT]);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const char angleID[] = ANGLE_ID_ARR;

static const uart_cfg_t configUART = {.baudrate=115200, .parity=NO_PARITY, .MSBF=false};


static const double gFactor = 488e-6;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/

static tim_id_t timerSens, timerUART;

static angle_t groupsValues[GROUPS_COUNT][AXIS_COUNT];		// Valores de todos los angulos

static CANMsg_t CANRx;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{

	timerInit();

	uartInit(UART_ID, configUART);

	timerSens = timerGetId();

	timerStart(timerSens, TIMER_MS2TICKS(TIME_SENS_CHECK), TIM_MODE_PERIODIC, NULL);
	timerStart(timerUART, TIMER_MS2TICKS(TIME_UART_SEND), TIM_MODE_PERIODIC, NULL);

}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	init_ACC_MAG();
	CANInit(CAN_ID, &CANRx);
	while (1) {
		if (newMsg()) {		// Se recibe mensaje CAN
			parseCANMsg(groupsValues[CANRx.ID - CAN_BASE_ID], (char*)CANRx.data, CANRx.length);
		}
		if (timerExpired(timerSens)) {
			checkSensorAndSend();
		}
		if (timerExpired(timerUART)) {
			sendAngleUART();
		}
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Funcion donde se guarda los valores recibidos al array
void parseCANMsg(angle_t angles[AXIS_COUNT], char* msg, size_t length) {

	uint8_t axis;

	// Chequeo del angleId (primer bit)
	for (int i = 0; i < AXIS_COUNT; i++) {

		if (*msg == angleID[i]) {
			axis = i;
			break;
		}
		else if (i == AXIS_COUNT-1) {
			return;				// No hago nada si el primer byte no corresponde
		}

	}

	char textToConvert[6];		// Nuevo string para agregar terminador
	memcpy(textToConvert, msg+1, length-1);	// Y poder usar atoi()
	textToConvert[length-1] = '\0';
	angles[axis] = atoi(textToConvert);		// Guarda el angulo

}


void checkSensorAndSend() {

	static uint32_t noSend[3];

	angle_t *myValues = groupsValues[GROUP_NUMBER];

	angle_t newValues[3];


	 readAngles(newValues);
	////////////////////////////	STUB
//	for (int i = 0; i < AXIS_COUNT; i++) {
//		newValues[i] = myValues[i] + i+1;
//	}
//		newValues[0] = myValues[0] + 1;
//		newValues[1] = myValues[1] + 3;
//		newValues[2] = myValues[2] + 10;
	///////////////////////////


	for (int i = 0; i < AXIS_COUNT; i++) {

		if (newValues[i] >= myValues[i] + ANGLE_DELTA || newValues[i] <= myValues[i] - ANGLE_DELTA || noSend[i] > MIN_SEND_CYCLES) {		// El eje se movio, o paso
																																			// tiempo sin mandar
			char msg[6];
			msg[0] = angleID[i];
			uint8_t size = sprintf(msg+1, "%d", myValues[i]) + 1;

			CANSend((uint8_t*)msg, size);
			// uartWriteMsg(UART_ID, msg, size);		// DEBUG

			noSend[i] = 0;		// Aviso que se mando
		}
		else {
			noSend[i]++;	// Otro ciclo sin mandar
		}
		myValues[i] = newValues[i];		// Actualizo
	}
}


void readAngles(angle_t anglesArr[AXIS_COUNT]) {

	ACCEL acc;
	MAG mg;

	getLastRead_ACC_MAG(&acc, &mg);

	anglesArr[0] = atan2(acc.x, sqrt(pow(acc.y,2) + pow(acc.z,2)))*(180.0/PI);

	anglesArr[1] = atan2(acc.y, acc.z)*(180.0/PI);

	acc.x *= gFactor;

	anglesArr[2] = asin(acc.x)*(180.0/PI);

	startReading_ACC_MAG();		// Lanzo lectura para el proximo ciclo

}

void sendAngleUART() {

	char msg[MAX_UART_MSG];

	size_t size = 0;
	for (int i = 0; i < GROUPS_COUNT; i++) {
		size = sprintf(msg, "%#X:\t%d\t%d\t%d\r\n", CAN_BASE_ID+i, groupsValues[i][0], groupsValues[i][1], groupsValues[i][2]);
		uartWriteMsg(UART_ID, msg, size);
	}

}

/*******************************************************************************
 ******************************************************************************/
