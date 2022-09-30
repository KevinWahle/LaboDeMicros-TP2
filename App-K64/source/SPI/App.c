/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "../MCAL/board.h"
#include "SPI.h"
#include <stdio.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
static SPI_config_t config;
static SPI_config_t * myconfig;

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
    gpioMode(PIN_SW3, INPUT);					//Ya es pulldown electricamente
    gpioMode(PIN_SW2, INPUT_PULLDOWN);					//Ya es pulldown electricamente

    myconfig=&config;
    myconfig->type=MASTER;
    myconfig->PCS_inactive_state=1;
    myconfig->LSB_fist=0;
    myconfig->frame_size=8;
    myconfig->clk_pol=0;
    myconfig->clk_phase=1;
    myconfig->Baud_rate_scaler=15;

    SPI_config(SPI_0,myconfig);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	//if(SPITransferCompleteFlag(SPI_0)){
    //SPIRead(SPI_0);
	//}

    

	if (!gpioRead(PIN_SW3)){
		while (!gpioRead(PIN_SW3));
			uint8_t msg[]="hola";
			//SPISend(SPI_0, msg, 0, 4);

	}

	if (!gpioRead(PIN_SW2)){
		while (!gpioRead(PIN_SW2));
			//printf("%c",SPIRead(SPI_0));

	}

  
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
