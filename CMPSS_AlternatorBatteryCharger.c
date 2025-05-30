#include <AlternatorBatteryChargerF280033.h>

void InitCmpss(void){
    EALLOW;

//+++++++++Firstly allocating comparator to A5 for the trip and current sensing++++++++
    AnalogSubsysRegs.CMPHPMXSEL.bit.CMP2HPMXSEL = 5;    //    A5 pin to positive high
    AnalogSubsysRegs.CMPLPMXSEL.bit.CMP2LPMXSEL = 5;    //    A5 pin to negative low

//++++++++++Configuring the source for the cmpss we are using++++++++++++
    Cmpss2Regs.COMPCTL.bit.COMPLSOURCE = 0;     //We want the source to be the DAC itself
    Cmpss2Regs.COMPCTL.bit.COMPHSOURCE = 0;     //We want here the source also the DAC

//+++++++++++Configuring whether to use the inverse input or not+++++++++++++
    Cmpss2Regs.COMPCTL.bit.COMPHINV = 0;        //Output of the comparator will be set high
    Cmpss2Regs.COMPCTL.bit.COMPLINV = 1;        //Output of the comparator is by default high and on the trip will be set to low

    Cmpss2Regs.COMPCTL.bit.CTRIPHSEL        = 2;    // CTRIPH = Output of digital filter
    Cmpss2Regs.COMPCTL.bit.CTRIPLSEL        = 2;    // CTRIPL = Output of digital filter

//++++++++++++We dont want to use the asysnchronous input but rather rely on the output pass by the digital filter++++++++++++++
    Cmpss2Regs.COMPCTL.bit.ASYNCHEN = 0;        //We are not willing to feed the asynchronous output but rather use the digital filter
    Cmpss2Regs.COMPCTL.bit.ASYNCLEN = 0;

//+++++++++++++++Using the typical hysteresis so that noise does not impact the performance of the comparator++++++++++++++
    Cmpss2Regs.COMPHYSCTL.bit.COMPHYS = 1;

//++++++++++++++Configuring the DAC to be fed to the negative input of the comparator of the cmpss2+++++++++++++++
    Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;    //The DACVALA will be directly fed form the DACVALS instead of the ramp generator
    Cmpss2Regs.COMPDACCTL.bit.SELREF = 0;       //VDDA=3.3v has been set as the reference voltage of the DAC
    Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;    //The DACVALS will be updated after sysclock cycle instead of any epwmsyncper event

//+++++++++++++Configuring the values of the DACs that we are using++++++++++++
    Cmpss2Regs.DACHVALS.bit.DACVAL = 2900;          // Trip TO 15A limit to avoid the overcurrent heating.
    Cmpss2Regs.DACLVALS.bit.DACVAL = 1500;

//++++++++++++Configuring the digital filter++++++++++++++

//++++++++++++For the COMPL comparator++++++++++++++++
    Cmpss2Regs.CTRIPLFILCLKCTL = 5;     //Fixing the clock to be used to be same as sysclock ie (5+1) CLK CYCLES
    Cmpss2Regs.CTRIPLFILCTL.bit.SAMPWIN = 8;  //The digital filter need (8+1) samples for the judgement of tripping
    Cmpss2Regs.CTRIPLFILCTL.bit.THRESH = 5;  //If  of the 9 samples (5+1) samples came out to be opposite the present state the tripping will occur
    Cmpss2Regs.CTRIPLFILCTL.bit.FILINIT = 1;    //Initialising the comparator subsystem
    Cmpss2Regs.CTRIPLFILCLKCTL = 5;
    Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1;     // Cleared the latch to allow the further outcomes to updates to latch the desired value
//    Cmpss2Regs.COMPSTS.bit.COMPLLATCH         //This is used to find the latched value and can help to figure out what caused the interrupt or the trip


//++++++++++++For the COMPH comparator++++++++++++++++
    Cmpss2Regs.CTRIPHFILCLKCTL = 5;     //Fixing the clock to be used to be same as sysclock ie (5+1) CLK CYCLES
    Cmpss2Regs.CTRIPHFILCTL.bit.SAMPWIN = 8;  //The digital filter need (8+1) samples for the judgement of tripping
    Cmpss2Regs.CTRIPHFILCTL.bit.THRESH = 5;  //If  of the 9 samples (5+1) samples came out to be opposite the present state the tripping will occur
    Cmpss2Regs.CTRIPHFILCTL.bit.FILINIT = 1;    //Initialising the comparator subsystem
    Cmpss2Regs.CTRIPLFILCLKCTL = 5;
    Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1;    //Previous value has been cleared
//    Cmpss2Regs.COMPSTS.bit.COMPHLATCH         //This bit will tell us which signal caused the tripping

//+++++++++++Configuring for the hardware tripping++++++++++++++++
    Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;    //Internal DAC is enabled for the cmpss2

    EDIS;
}
