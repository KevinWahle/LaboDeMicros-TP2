#include "FXOS8700CQ.h"
#include "I2Cm/I2Cm.h"
#include "timer/timer.h"
#include <stdbool.h>
#include "MCAL/gpio.h"
// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR 0x1D

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C
#define FXOS8700CQ_WHOAMI_VAL 0xC7

#define I2C_ID		I2C_ACC
#define I2C_ADDR	0x1D

typedef enum {READING, READY} READ_STATES;

static uint8_t writeBuffer [1];
static uint8_t readBuffer [13];

static READ_STATES readState = READY;

static ACCEL AccLastRead;
static MAG MagLastRead;

void FXOS8700CQ_init(){
	I2CmInit(I2C_ACC);
	uint8_t writeBuffer [2];
	uint8_t readBuffer [13];

	// write 0000 0000 = 0x00 to accelerometer control register 1 to
	//place FXOS8700CQ into
	// standby
	// [7-1] = 0000 000
	// [0]: active=0

	writeBuffer[0] = FXOS8700CQ_CTRL_REG1;
	writeBuffer[1] = 0x00;
	I2CmStartTransaction(I2C_ID, I2C_ADDR, writeBuffer, 2, readBuffer, 0);

	// write 0001 1111 = 0x1F to magnetometer control register 1
	// [7]: m_acal=0: auto calibration disabled
	// [6]: m_rst=0: no one-shot magnetic reset
	// [5]: m_ost=0: no one-shot magnetic measurement
	// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
	// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active

	while(isI2CBusy(I2C_ACC));
	writeBuffer[0] = FXOS8700CQ_M_CTRL_REG1;
	writeBuffer[1] = 0x1F;
	I2CmStartTransaction(I2C_ID, I2C_ADDR, writeBuffer, 2, readBuffer, 0);

	// write 0010 0000 = 0x20 to magnetometer control register 2
	// [7]: reserved
	// [6]: reserved
	// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to
	//follow the
	// accelerometer registers
	// [4]: m_maxmin_dis=0 to retain default min/max latching even
	//though not used
	// [3]: m_maxmin_dis_ths=0
	// [2]: m_maxmin_rst=0
	// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle

	while(isI2CBusy(I2C_ACC));
	writeBuffer[0] = FXOS8700CQ_M_CTRL_REG2;
	writeBuffer[1] = 0x20;
	I2CmStartTransaction(I2C_ID, I2C_ADDR, writeBuffer, 2, readBuffer, 0);
	// write 0000 0001= 0x01 to XYZ_DATA_CFG register
	// [7]: reserved
	// [6]: reserved
	// [5]: reserved
	// [4]: hpf_out=0
	// [3]: reserved
	// [2]: reserved
	// [1-0]: fs=01 for accelerometer range of +/-4g range with

	while(isI2CBusy(I2C_ACC));
	writeBuffer[0] = FXOS8700CQ_XYZ_DATA_CFG;
	writeBuffer[1] = 0x01;
	I2CmStartTransaction(I2C_ID, I2C_ADDR, writeBuffer, 2, readBuffer, 0);

	// write 0000 1101 = 0x0D to accelerometer control register 1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
	// [2]: lnoise=1 for low noise mode
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=1 to take the part out of standby and enable
	//sampling

	while(isI2CBusy(I2C_ACC));
	writeBuffer[0] = FXOS8700CQ_CTRL_REG1;
	writeBuffer[1] = 0x0D;
	I2CmStartTransaction(I2C_ID, I2C_ADDR, writeBuffer, 2, readBuffer, 0);

	while(isI2CBusy(I2C_ACC));
}

static bool checkForNewDataAvailableFXOS8700CQ(void){
	if(!isI2CBusy(I2C_ID)){
		readState = READY;
		return true;
	}
	else{
		readState = READING;   // ya se deberia saber esto pero por si acaso
	}
	return false;
}

void getLastReadFXOS8700CQ(ACCEL * acc, MAG * mg){

	checkForNewDataAvailableFXOS8700CQ(); // lo uso para actualizar readState... se podria hacer con IRQ
	if(readState == READY){
		// copy the 14 bit accelerometer byte data into 16 bit words
		acc->x = (int16_t)(((readBuffer[1] << 8) | readBuffer[2]))>> 2;
		acc->y = (int16_t)(((readBuffer[3] << 8) | readBuffer[4]))>> 2;
		acc->z = (int16_t)(((readBuffer[5] << 8) | readBuffer[6]))>> 2;
		// copy the magnetometer byte data into 16 bit words
		mg->x = (readBuffer[7] << 8) | readBuffer[8];
		mg->y = (readBuffer[9] << 8) | readBuffer[10];
		mg->z = (readBuffer[11] << 8) | readBuffer[12];

		AccLastRead = *acc;
		MagLastRead = *mg;
	}
	else{
		*acc = AccLastRead;
		*mg = MagLastRead;
	}
}


void startReadAccMagFXOS8700CQ(void){
	writeBuffer[0] = FXOS8700CQ_STATUS;
	I2CmStartTransaction(I2C_ID, I2C_ADDR, writeBuffer, 1, readBuffer, 13);

	readState = READING;
}





