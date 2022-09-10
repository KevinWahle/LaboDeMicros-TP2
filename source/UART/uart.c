/***************************************************************************//**
  @file		uart.c
  @brief	Funciones para comunicación por UART
  @author	Sergio Peralta
  @date		9 sep. 2022
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "uart.h"
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define CORE_CLOCK	1000000L

#define BUS_CLOCK	50000000L

#define DEFAULT_BAUDRATE	115200

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: unsigned int anio_actual;+


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

// +ej: static void falta_envido (int);+


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static UART_Type* const UARTPorts[] = UART_BASE_PTRS;

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// +ej: static int temperaturas_actuales[4];+


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/**
 * @brief Initialize UART driver
 * @param id UART's number
 * @param config UART's configuration (baudrate, parity, etc.)
*/
void uartInit (uint8_t id, uart_cfg_t config) {

	UART_Type* uart = UARTPorts[id];

	// Clock Gating

	uint32_t clkFreq = CORE_CLOCK;

	switch (id) {

		case 0:
			SIM->SCGC4 |= SIM_SCGC4_UART0(1);
			break;

		case 1:
			SIM->SCGC4 |= SIM_SCGC4_UART1(1);
			break;

		case 2:
			SIM->SCGC4 |= SIM_SCGC4_UART2(1);
			clkFreq = BUS_CLOCK;
			break;

		case 3:
			SIM->SCGC4 |= SIM_SCGC4_UART3(1);
			clkFreq = BUS_CLOCK;
			break;

		case 4:
			SIM->SCGC1 |= SIM_SCGC1_UART4(1);
			clkFreq = BUS_CLOCK;
			break;

		case 5:
			SIM->SCGC1 |= SIM_SCGC1_UART5(1);
			clkFreq = BUS_CLOCK;
			break;

		default:
			break;

	}


// Baud rate config

	uint32_t baudrate;
	uint16_t sbr, brfa;

	baudrate = config.baudrate;

	if (!baudrate) baudrate = DEFAULT_BAUDRATE;

	sbr = clkFreq / (baudrate << 4);

	brfa = (clkFreq << 1) / baudrate - (sbr << 5);

	// IMPORTANTE: Primero el HIGH
	uart->BDH = UART_BDH_SBNS(sbr >> 8);		// Borra los otros flags

	uart->BDL = UART_BDL_SBR(sbr);

	uart->C4 = UART_C4_BRFA(brfa);		// Borra los otros flags

// Control

	//Parity

	uart->C1 = UART_C1_PE(config.parity != NO_PARITY);	// Borra los demas flags

	uart->C1 = UART_C1_PE(config.parity == ODD_PARITY);	// Borra los demas flags

	// Habilitar Tx y Rx

	uart->C2 = UART_C2_TE(1) | UART_C2_RE(1);		// Borra los demas flags

}


/**
 * @brief Check if a new byte was received
 * @param id UART's number
 * @return A new byte has being received
*/
bool uartIsRxMsg(uint8_t id){
	return 0;
}


/**
 * @brief Check how many bytes were received
 * @param id UART's number
 * @return Quantity of received bytes
*/
uint8_t uartGetRxMsgLength(uint8_t id) {
	return 0;
}


/**
 * @brief Read a received message. Non-Blocking
 * @param id UART's number
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant) {
	return 0;
}


/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id UART's number
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg(uint8_t id, const char* msg, uint8_t cant) {
	return 0;
}


/**
 * @brief Check if all bytes were transfered
 * @param id UART's number
 * @return All bytes were transfered
*/
bool uartIsTxMsgComplete(uint8_t id) {
	return false;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



 
