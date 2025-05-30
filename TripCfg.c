#include <AlternatorBatteryChargerF280033.h>

void InitTripCfg(void){
//    Configuring the digital compare and the trip zone submodules for tripping of TRIP4
    asm(" EALLOW");                                      // Enable EALLOW protected register access
         EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL    = 3;          // select TRIPIN4 as source for DCAH
         EPwm1Regs.TZDCSEL.bit.DCAEVT1          = 2;          // define event as  : DCAH high
         EPwm1Regs.DCACTL.bit.EVT1SRCSEL        = 0;          // DCAEVT1 is selected as source
         EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL    = 0;          // passed asynchronously
         EPwm1Regs.DCACTL.bit.EVT1SOCE          = 0;          // SOC generation Disabled
         EPwm1Regs.DCACTL.bit.EVT1SYNCE         = 0;          // Synch disabled
         EPwm1Regs.DCACTL.bit.rsvd1             = 1;          // DCAEVT1 Latch enabled
         EPwm1Regs.TZCTL2.bit.ETZE              = 1;          // Use data from TZCTL2 and TZCTLDCA
         EPwm1Regs.TZCTL2.bit.TZAU              = 2;          // Force EPWMA to zero on UP count   if TZ1-TZ6 occurs
         EPwm1Regs.TZCTL2.bit.TZAD              = 2;          // Force EPWMA to zero on down count if TZ1-TZ6 occurs
         EPwm1Regs.TZCTL2.bit.TZBU              = 2;          // Force EPWMB to zero on UP count   if TZ1-TZ6 occurs
         EPwm1Regs.TZCTL2.bit.TZBD              = 2;          // Force EPWMB to zero on down count if TZ1-TZ6 occurs
         EPwm1Regs.TZCTLDCA.bit.DCAEVT1D        = 2;          // Force EPWMA to zero on down count if DCAEVT1 occurs
         EPwm1Regs.TZCTLDCA.bit.DCAEVT1U        = 2;          // Force EPWMA to zero on up count   if DCAEVT1 occurs
         EPwm1Regs.TZCTLDCB.bit.DCBEVT1D        = 2;          // Force EPWMB to zero on down count if DCAEVT1 occurs
         EPwm1Regs.TZCTLDCB.bit.DCBEVT1U        = 2;          // Force EPWMB to zero on up count   if DCAEVT1 occurs
         EPwm1Regs.TZSEL.bit.DCAEVT1            = 1;          // DCAEVT1 trip as One Shot
         EPwm1Regs.TZSEL.bit.OSHT1             = 1;          // TZ1 as one Shot
         EPwm1Regs.TZEINT.bit.OST               = 1;          // enable One Shot Trip interrupt
     asm(" EDIS");
}
