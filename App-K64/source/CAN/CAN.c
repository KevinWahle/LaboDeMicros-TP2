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
#include <string.h>
#include <stdbool.h>

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

#define RXB0D0_REG 0x66
#define RXB0D1_REG 0x67
#define RXB0D2_REG 0x68
#define RXB0D3_REG 0x69
#define RXB0D4_REG 0x6A
#define RXB0D5_REG 0x6B
#define RXB0D6_REG 0x6C
#define RXB0D7_REG 0x6D


#define TXRTSCTRL_REG 0x0D


#define WRITESIZE 3
#define READSIZE 3
#define BITMODSIZE 4

#define MAXBYTES 8
#define IRQ_CAN   PORTNUM2PIN(PA,24) // PTA4
/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
static CANMsg* MSGReceive; ******************************************************************************/

static SPI_config_t config;
static SPI_config_t * myconfig;

static uint16_t myID;

static uint8_t interrupt;
static uint8_t RXdlc;
static uint8_t RXIDL, RXIDH;
static uint8_t Rxdata[8];
static CANMsg_t* MSGReceive; 

static bool done;


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
void CANRead (uint8_t address, uint8_t* save, CBType mycb);
void CANBitModify(uint8_t address, uint8_t mask, uint8_t data);
void CANWrite (uint8_t address, uint8_t value);

void CANReceive();
void viewinterrupt();
void readData();
void readEnd ();


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

void CANInit(uint16_t ID, CANMsg_t* msgReceive){
    
    myID=ID; 
    MSGReceive=msgReceive;

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

void CANWrite (uint8_t address, uint8_t value){
  package data[WRITESIZE];
  
  data[0].msg=WRITE; data[0].pSave=NULL; data[0].cb=NULL ;data[0].read=false; data[0].cs_end=0;
  data[1].msg=address; data[1].pSave=NULL; data[1].cb=NULL; data[1].read=false; data[1].cs_end=0;
  data[2].msg=value; data[2].pSave=NULL; data[2].cb=NULL; data[2].read=false; data[2].cs_end=1;

  SPISend(SPI_0, data, WRITESIZE, 0);
}

void CANRead (uint8_t address, uint8_t* save, CBType mycb){
  package data[READSIZE];
  
  data[0].msg=READ; data[0].pSave=NULL; data[0].cb=NULL; data[0].read=false; data[0].cs_end=0;
  data[1].msg=address; data[1].pSave=NULL; data[1].cb=NULL; data[1].read=false; data[1].cs_end=0;
  //data[2].msg=0; data[2].pSave=NULL; data[2].cb=NULL; data[2].read=false;
  data[2].msg=0; data[2].pSave=save; data[2].cb=mycb; data[2].read=true; data[2].cs_end=1;

  SPISend(SPI_0, data, READSIZE, 0);
}


void CANBitModify(uint8_t address, uint8_t mask, uint8_t value){
  package data[BITMODSIZE];
  
  data[0].msg=BIT_MODIFY; data[0].pSave=NULL; data[0].cb=NULL; data[0].read=false; data[0].cs_end=0;
  data[1].msg=address; data[1].pSave=NULL; data[0].cb=NULL; data[1].read=false; data[1].cs_end=0;
  data[2].msg=mask; data[2].pSave=NULL; data[0].cb=NULL; data[2].read=false; data[2].cs_end=0;
  data[3].msg=value; data[3].pSave=NULL; data[0].cb=NULL; data[3].read=false; data[3].cs_end=1;
  SPISend(SPI_0, data, READSIZE, 0);
}


bool CANSend(uint16_t ID, uint8_t * data, uint8_t len){
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
  return true;
}

void CANReceive(){
  CANBitModify(CANINTF_REG, 0x01, 0x00); //Pongo el Flag en 0
  done=false; //Starting to receive
  CANRead(CANINTF_REG, &interrupt, viewinterrupt);
  
}

void viewinterrupt(){
  if ((interrupt%2)==1){  // Veo que sea la interrupcion que quiero (RX0IF)
    CANRead(RXB0DLC_REG, &RXdlc, readData); // Veo cuantos datos tengo
  }

}

void readData(){

  //Read data
  switch(RXdlc){
    case 8:
      CANRead(RXB0D7_REG, &Rxdata[7], NULL);
    case 7:
      CANRead(RXB0D6_REG, &Rxdata[6], NULL);
    case 6:
      CANRead(RXB0D5_REG, &Rxdata[5], NULL);
    case 5:
      CANRead(RXB0D4_REG, &Rxdata[4], NULL);
    case 4:
      CANRead(RXB0D3_REG, &Rxdata[3], NULL);
    case 3:
      CANRead(RXB0D2_REG, &Rxdata[2], NULL);
    case 2:
      CANRead(RXB0D1_REG, &Rxdata[1], NULL);
    case 1:
      CANRead(RXB0D0_REG, &Rxdata[0], NULL);
      break;
    default:
      break;
  }
    //Read ID 
    CANRead(RXB0SIDH_REG, &RXIDL, NULL);
    CANRead(RXB0SIDL_REG, &RXIDH, readEnd);

}

void readEnd (){
    MSGReceive->ID=RXIDH<<8|RXIDL;     //TODO: chequear
    MSGReceive->length=RXdlc;
    memcpy(MSGReceive->data, Rxdata, MAXBYTES);

    //Clear all
    RXIDL=0; RXIDH=0;
    RXdlc=0;
    for (int i=0; i<MAXBYTES; i++){
      Rxdata[i]=0;
    }

    done=true; //Reception done succesfull
}

bool newMsg(){
  bool newmsg=done;
  done = false;
  return newmsg;
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/





