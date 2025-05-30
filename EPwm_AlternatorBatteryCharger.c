

/*
 * Author: Varun Sharma
 * PWM Modules for Inverter Section : 1-3
 * Switching Frequency: 100 kHz
 */

/**********************************************************************

**********************************************************************/
#include <AlternatorBatteryChargerF280033.h>                        // Main include file

/**************************************************************
* Function: InitEPwm()
*
* Description: Initializes the Enhanced PWM modules on the F28x7x
**********************************************************************/
void InitEPwm(void)
{


// ################################################################################################
// ----- Must disable the clock to the ePWM modules if you want all ePWM modules synchronized. ----
// -------------------- Before Configuring the EPwm modules, set TBCLKSYNC = 0  -------------------
// ----------- After Configuring the EPwm modules, set TBCLKSYNC = 1 for synchronization ----------
// ################################################################################################

    asm(" EALLOW");  // Enable EALLOW protected register access

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    asm(" EDIS");    // Disable EALLOW protected register access


// ################################################################################################
// ------------------------ Configure ePWM1, ePWM2, ePWM3, and ePWM4 modules ----------------------
// ################################################################################################

    //****** Configure ePWM1, ePWM2, ePWM3, ePWM4, ePWM5, ePWM6 modules: START ******

     asm(" EALLOW");   // Enable EALLOW protected register access
     DevCfgRegs.SOFTPRES2.bit.EPWM1          = 1;     // ePWM1 is reset
     DevCfgRegs.SOFTPRES2.bit.EPWM1          = 0;     // ePWM1 is released from reset

     DevCfgRegs.SOFTPRES2.bit.EPWM2          = 1;     // ePWM2 is reset
     DevCfgRegs.SOFTPRES2.bit.EPWM2          = 0;     // ePWM2 is released from reset

     DevCfgRegs.SOFTPRES2.bit.EPWM3          = 1;     // ePWM3 is reset
     DevCfgRegs.SOFTPRES2.bit.EPWM3          = 0;     // ePWM3 is released from reset

     DevCfgRegs.SOFTPRES2.bit.EPWM4          = 1;     // ePWM4 is reset
     DevCfgRegs.SOFTPRES2.bit.EPWM4          = 0;     // ePWM4 is released from reset

     asm(" EDIS");     // Disable EALLOW protected register access

    //****** Configure ePWM1, ePWM2, ePWM3, ePWM4, ePWM5, ePWM6 modules: END ******

 // ################################################################################################
 // --------------------- ePWM1 Module Initialization : Phase-R ------------------
 // ################################################################################################

     //********** EPWM1 Initialization: START ***************

     EPwm1Regs.TBCTL.bit.CTRMODE            = 0x3;              // Disable the timer'
     EPwm1Regs.TBCTL.all                    = 0xC033;           // Configure timer control register

     // bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
     // bit 13        0:      PHSDIR, 0 = count down after sync event
     // bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1
     // bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = EPWMCLK/1
     // bit 6         0:      SWFSYNC, 0 = no software sync produced
     // bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
     // bit 3         0:      PRDLD, 0 = reload PRD on counter=0
     // bit 2         0:      PHSEN, 0 = phase control disabled
     // bit 1-0       11:     CTRMODE, 11 = timer stopped (disabled)

     EPwm1Regs.TBCTL.bit.HSPCLKDIV          = 0;
     EPwm1Regs.TBCTL.bit.CLKDIV             = 1;                   // Clock to EPWM Module is 60 MHz

     EPwm1Regs.TBCTL.bit.CTRMODE            = 0x02;                // Count Up-Down Mode

     EPwm1Regs.TBPRD                        = Tbprd;                 // 100 kHz switching frequency set

     EPwm1Regs.CMPA.bit.CMPA                = 0;                   // Set PWM duty cycle to 0 initially

     EPwm1Regs.CMPCTL.all                   = 0x0000;              // Shadowing of compare registers set to default

     EPwm1Regs.AQCTLA.all                   = 0x0000;              // Normal Sine-Triangle PWM

     EPwm1Regs.AQCTLA.bit.CAU               = 0x2;                 // Force EPWM1 to high as TBCTR = CMPA
     EPwm1Regs.AQCTLA.bit.CAD               = 0x1;                 // Force EPWM1 to low as t

     EPwm1Regs.ETSEL.bit.SOCAEN             = 1;                   // Enable SOCA
     EPwm1Regs.ETSEL.bit.SOCASEL            = 1;                   // SOCA on CTR = ZRO

     EPwm1Regs.ETPS.bit.SOCAPRD             = 3;                   // sampling frequency = (switching frequency) : 100/3 kHz

//+++++++++++++++Configuring the digital compare submodule for the tripping logic+++++++++++++++++++
     InitTripCfg();                                       // Disable EALLOW protected register access
//     InitTripCfgTrial();

     //********** EPWM1 Initialization: END ***************

         // ################################################################################################
         // ------------------------------- Phase Synchronization for EPWM Pins ---------------------------
         // ################################################################################################

//         EPwm1Regs.EPWMSYNCOUTEN.bit.ZEROEN    = 1;      //generate a synchout if CTR = 0 for EPWM1
//         EPwm1Regs.TBCTL2.bit.OSHTSYNCMODE     = 0;      //  OneShot sync mode disabled
//
//         EPwm2Regs.EPWMSYNCINSEL.bit.SEL       = 1;      // EWPM1SYNCHOUT = EPWM2SYNCHIN
//         EPwm2Regs.TBCTL.bit.PHSEN             = 1;      // enable phase shift for ePWM2
//         EPwm2Regs.TBCTL.bit.PHSDIR            = 1;      // count down/up after synchronization
//         EPwm2Regs.TBPHS.bit.TBPHS             = 0;      //  phase always same as EPWM 1
//
//         EPwm3Regs.EPWMSYNCINSEL.bit.SEL       = 1;      // EWPM1SYNCHOUT = EPWM2SYNCHIN
//         EPwm3Regs.TBCTL.bit.PHSEN             = 1;      // enable phase shift for ePWM2
//         EPwm3Regs.TBCTL.bit.PHSDIR            = 1;      // count down/up after synchronization
//         EPwm3Regs.TBPHS.bit.TBPHS             = 0;      //  phase always same as EPWM 1

     InitXBars();

// ------------------------------------------------------------------------------------------
// ----------------------------------------Interrupt Settings--------------------------------
// ------------------------------------------------------------------------------------------
         asm(" EALLOW");                              // Enable EALLOW protected register access
         PieCtrlRegs.PIEIER2.bit.INTx1=1;             // Enable Trip interrupt from EPwm1 (Trip ISR)
         IER |= 0x0002;

         asm(" EDIS");                                // Disable EALLOW protected register access

// ################################################################################################
//---------------- Enable the clocks to the ePWM module.
//---------------- Note: this should be done after all ePWM modules are configured
//---------------- to ensure synchronization between the ePWM modules.
// ################################################################################################

       //********** Enable TBCLK to ePWM Modules : START *************

       asm(" EALLOW");  // Enable EALLOW protected register access

       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // TBCLK to ePWM modules enabled

       asm(" EDIS");    // Disable EALLOW protected register access

       //********** Enable TBCLK to ePWM Modules : END *************

} // end InitEPwm()

//--- end of file -----------------------------------------------------



