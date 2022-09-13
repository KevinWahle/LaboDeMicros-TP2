/***************************************************************************//**
  @file     App.c
  @brief    TP2: Comunicacion Serie
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "UART/uart.h"
#include "MCAL/gpio.h"
#include "timer/timer.h"
#include "MCAL/board.h"
/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define UART_ID		0

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 *******************************************************************************
                        GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/

uint8_t cont = 0;

tim_id_t timer;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	gpioMode(PIN_LED_RED, OUTPUT);
	gpioMode(PIN_LED_GREEN, OUTPUT);

	gpioWrite(PIN_LED_RED, HIGH);
	gpioWrite(PIN_LED_GREEN, HIGH);

	uart_cfg_t cfg = {.MSBF = false, .baudrate = 9600, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
	
	timerInit();
	timer = timerGetId();
	uartWriteMsg(UART_ID, "Xola\r\n", 6);
	timerStart(timer, TIMER_MS2TICKS(500), TIM_MODE_PERIODIC, NULL);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	if (timerExpired(timer)) {
		if (uartIsTxMsgComplete(UART_ID)) {
			gpioToggle(PIN_LED_RED);
			char str[] = "Hola i\r\n";
			str[5] = '0' + cont++%10;
			uartWriteMsg(UART_ID, str, 8);
		}
	}

	while (uartIsRxMsg(UART_ID)) {
		char c;
		uartReadMsg(UART_ID, &c, 1);
		if (c == '3') {
			gpioWrite(PIN_LED_GREEN, LOW);
		}
		else if (c == '7') {
			gpioWrite(PIN_LED_GREEN, HIGH);
		}
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
