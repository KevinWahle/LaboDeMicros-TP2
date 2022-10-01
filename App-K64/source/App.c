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

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define UART_ID		0

#define ANGLE_DELTA	5	// °

#define TIME_SENS_CHECK	50	//ms

#define TIME_UART_SEND	100	//ms

#define MIN_SEND_CYCLES	20	// Ciclos maximos que pueden pasar sin mandar inclinacion

#define MAX_UART_MSG	32

#define GROUPS_COUNT	7

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

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const char angleID[] = ANGLE_ID_ARR;

static const uart_cfg_t configUART = {.baudrate=115200, .parity=NO_PARITY, .MSBF=false};

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

//	CANInit(CAN_ID, &CANRx);

	uartInit(UART_ID, configUART);

	timerSens = timerGetId();

	timerStart(timerSens, TIMER_MS2TICKS(TIME_SENS_CHECK), TIM_MODE_PERIODIC, NULL);
	timerStart(timerUART, TIMER_MS2TICKS(TIME_UART_SEND), TIM_MODE_PERIODIC, NULL);

}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	init_ACC_MAG();
	while (1) {
//		if (newMsg()) {		// Se recibe mensaje CAN
//			parseCANMsg(groupsValues[CANRx.ID - CAN_BASE_ID], CANRx.data, CANRx.length);
//		}
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

//	switch (*msg) {
//	case 'R':
//	case 'r':
//		axis = 0;
//		break;
//	case 'C':
//	case 'c':
//		axis = 1;
//		break;
//	case 'O':
//	case 'o':
//		axis = 2;
//		break;
//	default:
//		return;		// No hago nada si el primer byte no corresponde
//	}

	angles[axis] = atoi(msg+1);		// Guarda el angulo

}


void checkSensorAndSend() {

	static uint32_t noSend[3];

	angle_t *myValues = groupsValues[GROUP_NUMBER-1];

	angle_t newValues[3];


	// readValues(values);
	////////////////////////////	STUB
	for (int i = 0; i < AXIS_COUNT; i++) {
		newValues[i] = myValues[i] + 1;
	}
	///////////////////////////


	for (int i = 0; i < AXIS_COUNT; i++) {

		if (newValues[i] >= myValues[i] + ANGLE_DELTA || newValues[i] <= myValues[i] - ANGLE_DELTA || noSend[i] > MIN_SEND_CYCLES) {		// El eje se movio, o paso
																																			// tiempo sin mandar
			char msg[6];
			msg[0] = angleID[i];
			uint8_t size = sprintf(msg+1, "%d", myValues[i]) + 1;

//			CANSend(CAN_ID, msg, size);
			uartWriteMsg(UART_ID, msg, size);		// DEBUG

			noSend[i] = 0;		// Aviso que se mando
		}
		else {
			noSend[i]++;	// Otro ciclo sin mandar
		}
		myValues[i] = newValues[i];		// Actualizo
	}
}


void sendAngleUART() {

	char msg[MAX_UART_MSG];

	size_t size = 0;
	for (int i = 1; i <= GROUPS_COUNT; i++) {
		size = sprintf(msg, "%#X:\t%d\t%d\t%d\r\n", CAN_BASE_ID+i, groupsValues[i-1][0], groupsValues[i-1][1], groupsValues[i-1][2]);
		uartWriteMsg(UART_ID, msg, size);
	}

}

/*******************************************************************************
 ******************************************************************************/
