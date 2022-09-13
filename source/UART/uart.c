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
#include "hardware.h"
#include "buffer/circular_buffer.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define CORE_CLOCK	__CORE_CLOCK__

#define BUS_CLOCK	(CORE_CLOCK >> 1)

#define DEFAULT_BAUDRATE	115200

// Useful Macros

#define MIN(x, y)	((x) < (y) ? (x) : (y) )
#define MAX(x, y)	((x) > (y) ? (x) : (y) )

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

enum { PA, PB, PC, PD, PE };

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: unsigned int anio_actual;+


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void UART_Rx_Tx_IRQ(uint8_t id);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static UART_Type* const UARTPorts[] = UART_BASE_PTRS;
static PORT_Type* const portPtr[] = PORT_BASE_PTRS;

static uint32_t UARTClockSource[] = {CORE_CLOCK, CORE_CLOCK, BUS_CLOCK, BUS_CLOCK, BUS_CLOCK, BUS_CLOCK};

static __IO uint32_t* UARTSIMR[] = {&(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC1)};
static uint32_t UARTSIMMask[] = {SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART2_MASK, SIM_SCGC4_UART3_MASK, SIM_SCGC1_UART4_MASK};

static const IRQn_Type UART_RX_TX_Vectors[] = UART_RX_TX_IRQS;
//static const IRQn_Type UART_ERR_Vectors[] = UART_ERR_IRQS;

static const uint8_t UARTPinPort[] = {PB, PC, PD, PB, PE};

static const uint8_t UARTPinNumRX[] = {16, 3, 2, 10, 24};
static const uint8_t UARTPinNumTX[] = {17, 4, 3, 11, 25};
static const uint8_t UARTPinMuxAlt[] = {3, 3, 3, 3, 3};

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static circularBuffer TxBuffer[UART_CANT_IDS];
static circularBuffer RxBuffer[UART_CANT_IDS];

static bool TxOn[UART_CANT_IDS];

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void UART_Rx_Tx_IRQ(uint8_t id) {

	if(UARTPorts[id]->S1 & UART_S1_TDRE_MASK) {		// Interrupcion por buffer vacio
		if (!CBisEmpty(TxBuffer + id)) {
			uint8_t data = CBgetByte(TxBuffer + id);
			UARTPorts[id]->D = data;		// flag cleaned when writing in D buffer
		}
		else {
			// Turn off transmiter
			UARTPorts[id]->C2 &= ~UART_C2_TE(1);

//			UARTPorts[id]->D = 0;	//TODO: Ver si esta bien
			UARTPorts[id]->C2 &= ~UART_C2_TIE_MASK;		// Disable Interrupt	// TODO: Ver si esta bien


		}
	}
	if (UARTPorts[id]->S1 & UART_S1_RDRF_MASK) {	// Interrupcion por Receptor
		CBputByte(RxBuffer + id, UARTPorts[id]->D);
	}

}

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

	uint32_t clkFreq = UARTClockSource[id];
	*(UARTSIMR[id]) |= UARTSIMMask[id];


// Pinout setup

	//TODO: Enable CLK for PORTx
	portPtr[UARTPinPort[id]]->PCR[UARTPinNumRX[id]] = PORT_PCR_MUX(UARTPinMuxAlt[id]);		// Borra otros flags
	portPtr[UARTPinPort[id]]->PCR[UARTPinNumTX[id]] = PORT_PCR_MUX(UARTPinMuxAlt[id]);		// Borra otros flags

// Baud rate config

	uint32_t baudrate;
	uint16_t sbr, brfa;

	baudrate = config.baudrate;

	if (!baudrate) baudrate = DEFAULT_BAUDRATE;

	sbr = clkFreq / (baudrate << 4);
	brfa = (clkFreq << 1) / baudrate - (sbr << 5);

	// IMPORTANTE: Primero el HIGH
	uart->BDH = UART_BDH_SBR(sbr >> 8);			// Borra los otros flags
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = UART_C4_BRFA(brfa);				// Borra los otros flags
	uart->S2 = UART_S2_MSBF(config.MSBF);		// Borra los otros flags

// Enable NVIC Interrupts

	NVIC_EnableIRQ(UART_RX_TX_Vectors[id]);
	//NVIC_EnableIRQ(UART_ERR_Vectors[id]);

// Control

	// Parity (Si hay paridad, habilitar 9 bits de data)
	uart->C1 = UART_C1_PE(config.parity != NO_PARITY) | UART_C1_PT(config.parity == ODD_PARITY) | UART_C1_M(config.parity != NO_PARITY);	// Borra los demas flags
	// Habilitar Rx e interrupciones
	//uart->C2 = UART_C2_RE(1) | UART_C2_TIE(1) | UART_C2_TCIE(1) | UART_C2_RE(1);	// | UART_C2_TE(1) 		// Borra los demas flags
	uart->C2 = UART_C2_RIE_MASK | UART_C2_RE_MASK;
	TxOn[id] = false;
	// Habilita interrupcion por Parity Error y Overrun
	//uart->C3 = UART_C3_ORIE(1) | UART_C3_PEIE(1);		// Borra los otros flags


	////// TEST //////////
//	UARTPorts[id]->C1 |= UART_C1_LOOPS_MASK;


// Initialize Buffers
	CBinit(TxBuffer+id);
	CBinit(RxBuffer+id);

}



__ISR__ UART0_RX_TX_IRQHandler(void){
	UART_Rx_Tx_IRQ(0);
}

__ISR__ UART1_RX_TX_IRQHandler(void){
	UART_Rx_Tx_IRQ(1);
}

__ISR__ UART2_RX_TX_IRQHandler(void){
	UART_Rx_Tx_IRQ(2);
}

__ISR__ UART3_RX_TX_IRQHandler(void){
	UART_Rx_Tx_IRQ(3);
}

__ISR__ UART4_RX_TX_IRQHandler(void){
	UART_Rx_Tx_IRQ(4);
}


/**
 * @brief Check if a new byte was received
 * @param id UART's number
 * @return A new byte has being received
*/
bool uartIsRxMsg(uint8_t id){
	return !CBisEmpty(RxBuffer+id);
}


/**
 * @brief Check how many bytes were received
 * @param id UART's number
 * @return Quantity of received bytes
*/
uint8_t uartGetRxMsgLength(uint8_t id) {
	return CBgetBufferState(RxBuffer+id);
}


/**
 * @brief Read a received message. Non-Blocking
 * @param id UART's number
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant) {
	
	uint8_t i = 0;

	while (i++ < cant && CBgetBufferState(RxBuffer+id)) {
		*(msg++) = CBgetByte(RxBuffer+id);			// Copy to msg
	}

	return cant;

}


/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id UART's number
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg(uint8_t id, const char* msg, uint8_t cant) {

	CBputChain(TxBuffer+id, msg, cant);

	// TxOn[id] = true;

	UARTPorts[id]->D = CBgetByte(TxBuffer+id);

	UARTPorts[id]->C2 |= UART_C2_TE_MASK | UART_C2_TIE_MASK;


	return CBgetBufferState(TxBuffer+id);

}


/**
 * @brief Check if all bytes were transfered
 * @param id UART's number
 * @return All bytes were transfered
*/
bool uartIsTxMsgComplete(uint8_t id) {
	return CBisEmpty(TxBuffer+id);
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



 
