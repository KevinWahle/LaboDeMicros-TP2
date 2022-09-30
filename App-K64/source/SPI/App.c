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
#include "../buffer/SPI_buffer.h"
#include <stdio.h>
#include <stdint.h>

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

static package mypkg[3];

uint8_t first;
uint8_t second;

void myreadCB();

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

    

	if (!gpioRead(PIN_SW3)){            //Escribo
		while (!gpioRead(PIN_SW3));
			mypkg[0].msg = 'a';
            mypkg[0].pSave = NULL;
            mypkg[0].cb = NULL;
            mypkg[0].read = 0;
            mypkg[0].cs_end = 0; 

            mypkg[1].msg = 'b';
            mypkg[1].pSave = NULL;
            mypkg[1].cb = NULL;
            mypkg[1].read = 0;
            mypkg[1].cs_end = 0;

            mypkg[2].msg = 'f';
            mypkg[2].pSave = NULL;
            mypkg[2].cb = 0;
            mypkg[2].read = 0;
            mypkg[2].cs_end = 1;

            SPISend(SPI_0, mypkg, 3, 0);

			mypkg[0].msg = 'a';
            mypkg[0].pSave = NULL;
            mypkg[0].cb = NULL;
            mypkg[0].read = 0;
            mypkg[0].cs_end = 0;

            mypkg[1].msg = 'b';
            mypkg[1].pSave = &first;
            mypkg[1].cb = NULL;
            mypkg[1].read = 1;
            mypkg[1].cs_end = 0;

            mypkg[2].msg = 'f';
            mypkg[2].pSave = &second;
            mypkg[2].cb = myreadCB;
            mypkg[2].read = 1;
            mypkg[2].cs_end = 1;

            SPISend(SPI_0, mypkg, 3, 0);

	}

	if (!gpioRead(PIN_SW2)){
		while (!gpioRead(PIN_SW2));     //Escribo, leo, leo
			mypkg[0].msg = 'a';
            mypkg[0].pSave = NULL;
            mypkg[0].cb = NULL;
            mypkg[0].read = 0;
            mypkg[0].cs_end = 0; 

            mypkg[1].msg = 'b';
            mypkg[1].pSave = &first;
            mypkg[1].cb = NULL;
            mypkg[1].read = 1;
            mypkg[1].cs_end = 0;

            mypkg[2].msg = 'f';
            mypkg[2].pSave = &second;
            mypkg[2].cb = myreadCB;
            mypkg[2].read = 1;
            mypkg[2].cs_end = 1;

	}

  
}

void myreadCB(){
    printf("%c, %c", first, second);
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
