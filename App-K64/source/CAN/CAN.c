/***************************************************************************//**
  @file		CAN.c
  @brief	HAL para comunicación CAN
  @author	Grupo 5
  @date		19 sep. 2022
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "../SPI/SPI.h"
#include "CAN.h"
#include "../buffer/SPI_buffer.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
//Instructiones
#define READ 0x03
#define WRITE 0x02
#define BIT_MODIFY 0x05

//Registros
#define CNF1_REG 0x2A
#define CNF2_REG 0x29
#define CNF3_REG 0x28

#define CANCTRL_REG 0x0F
#define CANINTF_REG 0x2C


//Defines de direcciones de registros
#define CNF1_REG 0x2A
#define CNF2_REG 0x29
#define CNF3_REG 0x28

#define RXM0SIDH_REG 0x20
#define RXM0SIDL_REG 0x21

#define RXF0SIDH_REG 0x00
#define RXF0SIDL_REG 0x01

#define RXB0CTRL_REG 0x60
#define BFPCTRL_REG 0x0C
#define RXB0SIDH_REG 0x61
#define RXB0SIDL_REG 0x62
#define RXB0DLC_REG	0x65

#define CANCTRL_REG 0x0F
#define CANINTE_REG 0x2B
#define CANINTF_REG 0x2C

#define TXB0CTRL_REG 0x30
#define TXB0SIDH_REG 0x31
#define TXB0SIDL_REG 0x32
#define TXB0DLC_REG 0x35
#define TXB0D0_REG 0x36

#define TXRTSCTRL_REG 0x0D


#define WRITESIZE 3
#define READSIZE 4
#define BITMODSIZE 4

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

static SPI_config_t config;
static SPI_config_t * myconfig;

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/



/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/



/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
void CANInit(){
    CAN_bit_modify();

    //Poner en config mode ===> poner REQOP de CANCTRL en 100. 
    
    //Limpiar TXflag y RxFlag ===> Bits 2 y 0 del CANINTF
    
    //Setear Tiempos(Pag 24):
    // BRP (CNF1)     Valor 0
    // PRSEG (CNF2)   Valor 5
    // PHSEG1(CNF2)   Valor 5
    // PHSEG2(CNF3)   Valor 5
    // SJW(CNF1)      Valor 4

    // Setear filtros Cap 4.3
    
    // Config Recepción Cap 4.2

    // Poner Normal Mode CANCTRL cap 4.7  

    // CNF1: baudrateprescaler=3 (por consigna 125kbits/seg)

    SPI_init();
}

void SPI_init(){
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

void CANWrite (uint8_t address, uint8_t value){
  package data[WRITESIZE];
  
  data[0].msg=WRITE; data[0].pSave=NULL; data[0].read=false;
  data[1].msg=address; data[1].pSave=NULL; data[1].read=false;
  data[2].msg=value; data[2].pSave=NULL; data[2].read=false;

  SPISend(SPI_0, data, WRITESIZE, 0);
}

void CANRead (uint8_t address, uint8_t* save){
  package data[READSIZE];
  
  data[0].msg=READ; data[0].pSave=NULL; data[0].read=false;
  data[1].msg=address; data[1].pSave=NULL; data[1].read=false;
  data[2].msg=0; data[2].pSave=NULL; data[2].read=false;
  data[3].msg=0; data[3].pSave=save; data[3].read=true;

  SPISend(SPI_0, data, READSIZE, 0);


}

void CANBitModify(uint8_t address, uint8_t mask, uint8_t data){
  package data[BITMODSIZE];
  
  data[0].msg=BIT_MODIFY; data[0].pSave=NULL; data[0].read=false;
  data[1].msg=address; data[1].pSave=NULL; data[1].read=false;
  data[2].msg=mask; data[2].pSave=NULL; data[2].read=false;
  data[3].msg=data; data[3].pSave=NULL; data[3].read=false;
  SPISend(SPI_0, data, READSIZE, 0);
}
/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/





