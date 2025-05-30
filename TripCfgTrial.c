#include <AlternatorBatteryChargerF280033.h>

void InitTripCfgTrial(void){
//    Configuring the digital compare and the trip zone submodules for tripping of TRIP4

    asm(" EALLOW");                                      // Enable EALLOW protected register access
         EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL    = 3;          // select TRIPIN4 as source for DCAH
         EPwm1Regs.TZDCSEL.bit.DCAEVT1          = 2;          // dcbevt2 is the event for the tripping operation triggering
         EPwm1Regs.DCACTL.bit.EVT1SRCSEL        = 0;          // unfiltered dcbevt1 is the event triggered
         EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL    = 1;          // DCAEVT1 is selected as source
//         EPwm1Regs.DCACTL.bit.EVT1SOCE          = 0;          // no soc will be generated
//         EPwm1Regs.DCACTL.bit.EVT1SYNCE         = 0;          // no synchronisation pulse needed
//         EPwm1Regs.DCACTL.bit.EVT1LATSEL        = 1;          // source of the dcbevt1.force will be the latched dcbect1
//         EPwm1Regs.DCACTL.bit.EVT1LAT           = 0;          // previous value of the latch is cleared
         EPwm1Regs.TZCTL.bit.DCAEVT1            = 2;
         EPwm1Regs.TZCTL.bit.TZA                = 2;
         EPwm1Regs.TZSEL.bit.CBC4               = 1;
         EPwm1Regs.TZEINT.bit.CBC               = 1;          // enable One Shot Trip interrupt
     asm(" EDIS");
}
