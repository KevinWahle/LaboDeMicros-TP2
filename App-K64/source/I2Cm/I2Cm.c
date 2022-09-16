/***************************************************************************//**
  @file		I2Cm.c
  @brief	
  @author	Grupo 5
  @date		13 sep. 2022
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "I2Cm.h"
#include "MCAL/gpio.h"
#include "MK64F12.h"
#include "hardware.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// TODO: En modo MASTER RX, el valor a responder en el ACKbit se debe configurar antes de comenzar la lectura delbyte

#define I2C_COUNT	3

#define BUS_CLK	50000000UL

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

static void I2C_IRQ();

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static I2C_Type* const I2CPtrs[] = I2C_BASE_PTRS;
static IRQn_Type const I2CIRQs[] = I2C_IRQS;

static PORT_Type* const portPtr[] = PORT_BASE_PTRS;

static const uint8_t I2CPinPorts[] =    { PB, PC, PA, PE };
static const uint8_t I2CPinPinsSCL[] =  { 2,  10, 12, 24 };
static const uint8_t I2CPinPinsSDA[] =  { 3,  11, 13, 25 };
static const uint8_t I2CPinAlts[] =     { 2,  2,  5,  5  };

static __IO uint32_t* const I2CClkSimPtr[] = {&(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC1), &(SIM->SCGC4)};
static const uint32_t I2CClkSimMask[] = {SIM_SCGC4_I2C0_MASK, SIM_SCGC4_I2C1_MASK, SIM_SCGC1_I2C2_MASK, SIM_SCGC4_I2C0_MASK};


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/**
 * @brief Inicializa el modulo I2C
 * @param id: Instancia del I2C [0 - 1]
*/
void I2CmInit(I2CPort_t id) {

// Clock Gating

	//TODO: Enable CLK for PORTx
	*(I2CClkSimPtr[id]) |= I2CClkSimMask[id];

// Config pins (ALT, Open Drain, NO Pullup)

//	portPtr[I2CPinPorts[id]]->PCR[I2CPinPinsSCL[id]] = PORT_PCR_MUX(I2CPinAlts[id]) | PORT_PCR_ODE_MASK;
//	portPtr[I2CPinPorts[id]]->PCR[I2CPinPinsSDA[id]] = PORT_PCR_MUX(I2CPinAlts[id]) | PORT_PCR_ODE_MASK;

	// TEST:
	portPtr[I2CPinPorts[id]]->PCR[I2CPinPinsSCL[id]] = PORT_PCR_MUX(I2CPinAlts[id]) | PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	portPtr[I2CPinPorts[id]]->PCR[I2CPinPinsSDA[id]] = PORT_PCR_MUX(I2CPinAlts[id]) | PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

// I2C Master config

	// Clock Divider
	I2CPtrs[id%I2C_COUNT]->F = I2C_F_ICR(0x2B);		// Divider to 512 with no multiplier	(SCL: 100k)

	// Enable I2C. No Enable Master Mode and interrupts yet
	//	I2CPtrs[id%I2C_COUNT]->C1 = I2C_C1_IICEN_MASK;

	// Enable IRQ in NVIC

	NVIC_EnableIRQ(I2CIRQs[id%I2C_COUNT]);

}

/**
 * @brief realiza una transmision y recepcion po I2C
 * @param address address del slave
 * @param writeBuffer buffer de escritura
 * @param writeSize Tamano del buffer de escritura
 * @param readBuffer buffer para guardar la lectura
 * @param readSize Tamano del buffer de lectura
*/
void I2CmStartTransaction(I2CPort_t id, uint8_t address, uint8_t* writeBuffer, uint8_t writeSize, uint8_t* readBuffer, uint8_t readSize) {

	I2C_Type* pI2C = I2CPtrs[id%I2C_COUNT];

  // Initialize RAM variables

	// Enable 12C in Master Mode, transmit mode and interrupts
	pI2C->C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_IICIE_MASK;

	pI2C->D = address << 1;		// Slave Address + Write (0)

}



/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


__ISR__ I2C0_IRQHandler() {
	I2C_IRQ();
}

__ISR__ I2C1_IRQHandler() {
	I2C_IRQ();
}

__ISR__ I2C2_IRQHandler() {
	I2C_IRQ();
}


static void I2C_IRQ() {

}



 
