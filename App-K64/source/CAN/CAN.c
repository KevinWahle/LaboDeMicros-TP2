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
#include "../MCAL/gpio.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
//Instructiones
#define READ 0x03
#define WRITE 0x02
#define BIT_MODIFY 0x05

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
#define TXB0D1_REG 0x37
#define TXB0D2_REG 0x38
#define TXB0D3_REG 0x39
#define TXB0D4_REG 0x3A
#define TXB0D5_REG 0x3B
#define TXB0D6_REG 0x3C
#define TXB0D7_REG 0x3D

#define TXRTSCTRL_REG 0x0D


#define WRITESIZE 3
#define READSIZE 4
#define BITMODSIZE 4

#define MAXBYTES 8
#define IRQ_CAN   PORTNUM2PIN(PA,24) // PTA4
/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

static SPI_config_t config;
static SPI_config_t * myconfig;

uint16_t myID;

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
void CANInit(uint16_t ID){
    
    myID=ID; 

    gpioMode(IRQ_CAN, INPUT);
    gpioIRQ(IRQ_CAN, GPIO_IRQ_MODE_RISING_EDGE, CANReceive);   //TODO: Escribirla

    CANBitModify(CANCTRL_REG, 0xE0, 0x80); // Sets Configuration mode

    //TODO:
    // CANWrite(RXF0SIDH_REG, );
    // CANWrite(RXF0SIDL_REG, );
    // CANWrite(RXM0SIDH_REG, );
    // CANWrite(RXM0SIDL_REG, );

    CANWrite(TXRTSCTRL_REG,0x01); //Pin is used to request message transmission of TXB0 buffer (on falling edge)
    CANWrite(RXB0CTRL_REG,0x60);  //Turns mask/filters off; receives any message. TODO: Hay que recibir solos los de ID en un rango revisar RXM

    CANWrite(CNF1_REG,0xC0); //SJW=4 BRP=0
    CANBitModify(CNF2_REG,0x3F,0x2D); //PHSEG1=5 PRSEG=5      TODO:  toco el bltmode?
    CANBitModify(CNF3_REG,0x07,0x05); //PHSEG2=5

    //TODO:Hay que decidir cual de los dos usar
    CANWrite(CANINTE_REG,0x05); //TX0IE: Transmit Buffer 0 Empty Interrupt Enable bit
                                //RX0IE: Receive Buffer 0 Full Interrupt Enable bit
    CANWrite(CANINTF_REG,0x05); //Transmit Buffer 0 Empty Interrupt Flag bit
                                //RX0IF: Receive Buffer 0 Full Interrupt Flag bit

    //TODO: Puse el preescaler en 1. Si ponemos shooteo hasta que se manda el msg?
    CANBitModify(CANCTRL_REG, 0xEF, 0x04); //Sets Normal Operation mode,One-Shot, Clock Enable and Preescaler

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


bool CANSend(uint16_t ID, char * data, uint8_t len){
  if (len>MAXBYTES){
    return false;
  }

  CANBitModify(TXB0DLC_REG, 0x0F, len); //Set Length (DLC)
  
  //Set ID
  uint8_t IDH=(uint8_t) ID>>3;
  uint8_t IDL=(uint8_t) ((ID & 0x07) << 5);
  CANBitModify(TXB0SIDH_REG, 0xFF,IDH); 
  CANBitModify(TXB0SIDL_REG, 0XE0 ,IDL);

  //Set Data
  switch(len){
    case 8:
      CANWrite(TXB0D7_REG, data[7]);
    case 7:
      CANWrite(TXB0D6_REG, data[6]);
    case 6:
      CANWrite(TXB0D5_REG, data[5]);
    case 5:
      CANWrite(TXB0D4_REG, data[4]);
    case 4:
      CANWrite(TXB0D3_REG, data[3]);
    case 3:
      CANWrite(TXB0D2_REG, data[2]);
    case 2:
      CANWrite(TXB0D1_REG, data[1]);
    case 1:
      CANWrite(TXB0D0_REG, data[0]);
      break;

    default:
      break;
  }

  CANBitModify(TXB0CTRL_REG, 0x08 ,0x08); //Buffer is currently pending transmission
  //TODO: B0RTSM? B0BFM?

}

void CANReceive(){
  

}
  
/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/





