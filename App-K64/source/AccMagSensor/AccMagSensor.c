#include "FXOS8700CQ/FXOS8700CQ.h"

void init_ACC_MAG(){
	FXOS8700CQ_init();
}
void startReading_ACC_MAG(void){
	startReadAccMagFXOS8700CQ();
}
void getLastRead_ACC_MAG(ACCEL * acc, MAG * mg){
	getLastReadFXOS8700CQ(acc, mg);
}
