#include <AlternatorBatteryChargerF280033.h>         // Main include file
#include "math.h"

void InitSwitchesXBar(void){

    EALLOW;

    InputXbarRegs.INPUT4SELECT = 22;  //Goes to Xint1
    InputXbarRegs.INPUT5SELECT = 23;  //Goes to Xint2
    InputXbarRegs.INPUT6SELECT = 40;  //Goes to Xint3
    InputXbarRegs.INPUT13SELECT = 41; //Goes to Xint4

//    Doing the configuration for the external interrupts
//    Enabling all the interrupts
    XintRegs.XINT1CR.bit.ENABLE = 1;
    XintRegs.XINT2CR.bit.ENABLE = 1;
    XintRegs.XINT3CR.bit.ENABLE = 1;
    XintRegs.XINT4CR.bit.ENABLE = 1;

//    Also doing the configuration for the cause of the interrupt
    XintRegs.XINT1CR.bit.POLARITY = 0;
    XintRegs.XINT2CR.bit.POLARITY = 0;
    XintRegs.XINT3CR.bit.POLARITY = 0;
    XintRegs.XINT4CR.bit.POLARITY = 0;

    EDIS;
}
