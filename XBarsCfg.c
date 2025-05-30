#include <AlternatorBatteryChargerF280033.h>

void InitXBars(void){
    EALLOW;
//    Configuring the TRIP4 for generating the trip signals
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2 = 1;     //Group containing CMPSS2_CTRIPH is enabled
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3 = 1;     //Group containing CMPSS2_CTRIPL is enabled

    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 0;   //CMPSS2_CTRIPH is enabled
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 0;   //CMPSS2_CTRIPH is enabled

    EPwmXbarRegs.TRIPOUTINV.bit.TRIP4 = 0;      //As soon as the signal goes high the trip command is generated
    EDIS;
}
