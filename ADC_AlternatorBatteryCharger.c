//###########################################################################

//Author: Varun Sharma
//ADC pins used: A5, B10, C4, A7, B7, C1
//Triggering epwm: EPwm1 (SOC A)@100/3 kHz
//Interrrupt ISR used: ADCA1

//###########################################################################
#include <AlternatorBatteryChargerF280033.h>                        // Main include file
#include "f28003x_device.h"      // Header File Include File
#include "f28003x_examples.h"    // Examples Include File
#include "f28x_project.h"
//
//
// SetVREF - Set Vref mode. Function to select reference mode and offset trim.
// Offset trim for Internal VREF 3.3 is unique.  All other modes use the same
// offset trim. Also note that when the mode parameter is ADC_EXTERNAL, the
// ref parameter has no effect.
//
// In this device, the bandgaps are common for all the ADC instances,
// hence common Vref configuration needs to be done for all the ADCs. This
// API configures same Vref configuration for all the supported ADCs in the
// device.
//

// ADC Pins
/*
   ADC A6 - input line to line voltage (Vry)                              AIO 228
   ADC B2/C6 - input lint to line voltage (Vyb)                           AIO 226
   ADC A2/B6/C9 - DC Link Voltage                                         AIO 224
   ADC A5/B12/C2 - Output AC Current (ir)                                 AIO 244/249
   ADC A1/B7 - Output AC Current (iy)                                     AIO 232
   ADC A12/C1 - Output AC Current (ib)                                    AIO 238/248
   ADC A7/C3 - Analog input for VFD frequency and voltage setting         AIO 245
   ADC A11/B1/C0 - Power Semiconductor Module Temperature sensing         AIO 237
   ADC A14/B14/C4 - Voltage for fan failure detection                     AIO 239
*/

void SetVREF(int module, int mode, int ref)
{
    Uint16 *offset, offsetShiftVal;

    if((mode == ADC_INTERNAL) && (ref == ADC_VREF3P3))
    {
        offsetShiftVal = 8U;         // Internal / 1.65v mode offset
    }
    else
    {
        offsetShiftVal = 0U;        // All other modes
    }

    //
    // Set up pointer to offset trim in OTP for ADCA.
    //
    offset = (Uint16 *)((Uint32)0x7016CU);

    EALLOW;

    //
    // Get offset trim from OTP and write it to the register for ADCA.
    //
    AdcaRegs.ADCOFFTRIM.bit.OFFTRIM = (*offset >> offsetShiftVal) & 0xFFU;

    //
    // Set up pointer to offset trim in OTP for ADCB.
    //
    offset = (Uint16 *)((Uint32)0x7016DU);

    //
    // Get offset trim from OTP and write it to the register for ADCB.
    //
    AdcbRegs.ADCOFFTRIM.bit.OFFTRIM = (*offset >> offsetShiftVal) & 0xFFU;

    //
    // Set up pointer to offset trim in OTP for ADCC.
    //
    offset = (Uint16 *)((Uint32)0x7016EU);

    //
    // Get offset trim from OTP and write it to the register for ADCC.
    //
    AdccRegs.ADCOFFTRIM.bit.OFFTRIM = (*offset >> offsetShiftVal) & 0xFFU;

    //
    // Configure the reference mode for all ADCs (internal or external).
    //
    AnalogSubsysRegs.ANAREFCTL.bit.ANAREFSEL = mode;

    //
    // Configure the reference voltage for all ADCs (3.3V or 2.5V).
    //
    AnalogSubsysRegs.ANAREFCTL.bit.ANAREF2P5SEL = ref;

    EDIS;
}

void initADC(void)
{


    asm(" EALLOW");                     // Enable EALLOW protected register access

// ################################################################################################
// --------------------------------   Resetting ADCs - A, B and C  -----------------------------------
// ################################################################################################

    //******* Resetting the ADCs A, B, C: START *******

    DevCfgRegs.SOFTPRES13.bit.ADC_A         = 1;    // ADC A is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_A         = 0;    // ADC A is released from reset

    DevCfgRegs.SOFTPRES13.bit.ADC_B         = 1;    // ADC B is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_B         = 0;    // ADC B is released from reset

    DevCfgRegs.SOFTPRES13.bit.ADC_C         = 1;    // ADC C is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_C         = 0;    // ADC C is released from reset

    //******** Resetting the ADCs A, B, C: END ********

// ################################################################################################
// ----------------------------    Configure the ADC base register  -------------------------------
// ################################################################################################

    // ------ Configuring ADC A, ADC B, ADC C Control-1 Register: START -------

    AdcaRegs.ADCCTL1.all                    = 0x0004;      // Main ADC A configuration
    AdcbRegs.ADCCTL1.all                    = 0x0004;      // Main ADC B configuration
    AdccRegs.ADCCTL1.all                    = 0x0004;      // Main ADC C configuration

    // bit 15-14     00:     reserved
    // bit 13        0:      ADCBSY, ADC busy, read-only
    // bit 12        0:      reserved
    // bit 11-8      0's:    ADCBSYCHN, ADC busy channel, read-only
    // bit 7         0:      ADCPWDNZ, ADC power down, 0=powered down, 1=powered up
    // bit 6-3       0000:   reserved
    // bit 2         1:      INTPULSEPOS, INT pulse generation, 0=start of conversion, 1=end of conversion
    // bit 1-0       00:     reserved

    // ------ Configuring ADC A, ADC B, ADC C Control-1 Register: END -------

    // ------ Configuring ADC A, ADC B, ADC C Control-2 Register: START ------

    AdcaRegs.ADCCTL2.all                    = 0x0006;      // ADC A clock configuration (Set to 30MHz)
    AdcbRegs.ADCCTL2.all                    = 0x0006;      // ADC A clock configuration (Set to 30MHz)
    AdccRegs.ADCCTL2.all                    = 0x0006;      // ADC C clock configuration (Set to 30MHz)

    // bit 15-8      0's:    reserved
    // bit 7         0:      reserved
    // bit 6         0:      reserved
    // bit 5-4       00:     reserved
    // bit 3-0       0110:   PRESCALE, ADC clock prescaler.  0110=CPUCLK/4 = 200MHz/4 = 50MHz

    // ------- Configuring ADC A, ADC B, ADC C Control-2 Register: END -------

    // ------ Configuring ADC A, ADC B ADC C Burst Control Register (ADCBURSTCTL): START ------

    AdcaRegs.ADCBURSTCTL.all                = 0x0000;
    AdcbRegs.ADCBURSTCTL.all                = 0x0000;
    AdccRegs.ADCBURSTCTL.all                = 0x0000;

    // bit 15        0:      BURSTEN, 0=burst mode disabled, 1=burst mode enabled
    // bit 14-12     000:    reserved
    // bit 11-8      0000:   BURSTSIZE, 0=1 SOC converted (don't care)
    // bit 7-6       00:     reserved
    // bit 5-0       000000: BURSTTRIGSEL, 00=software only (don't care)

    // ------- Configuring ADC A, ADC B, ADC C Burst Control Register (ADCBURSTCTL): END -------

    asm(" EDIS");                               // Disable EALLOW protected register access

    // --------  Set Vref as Internal with 0 to 3.3V Range: START ----------

    SetVREF(ADC_ADCA, ADC_EXTERNAL, ADC_VREF3P3);
    SetVREF(ADC_ADCB, ADC_EXTERNAL, ADC_VREF3P3);
    SetVREF(ADC_ADCC, ADC_EXTERNAL, ADC_VREF3P3);

// ################################################################################################
// -------------------   SOC Configure for the 4 ADC Groups (A, B, C, D)   ------------------------
// ################################################################################################

    asm(" EALLOW");                     // Enable EALLOW protected register access

    //----- SOC (SOC0, SOC1, SOC2) Configuration for ADC A: START -----

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL           = 0x05;    // Convert channel ADC A5: ie ac current
    AdcaRegs.ADCSOC0CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns

    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
    AdcaRegs.ADCSOC1CTL.bit.CHSEL           = 0x07;    // Convert channel ADC A7: For sensing the temperature of the heat sink
    AdcaRegs.ADCSOC1CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns
//
//    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
//    AdcaRegs.ADCSOC2CTL.bit.CHSEL           = 0x06;    // Convert channel ADC A6: Input line to line voltage (vry)
//    AdcaRegs.ADCSOC2CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns
//
//    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
//    AdcaRegs.ADCSOC3CTL.bit.CHSEL           = 0x0C;    // Convert channel ADC A12: Y phase output current (iy)
//    AdcaRegs.ADCSOC3CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns
//----- SOC (SOC0, SOC1, SOC2) Configuration for ADC A: END -----

//----- SOC (SOC0, SOC1, SOC2) Configuration for ADC B: START -----

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
    AdcbRegs.ADCSOC0CTL.bit.CHSEL           = 0x0A;    // Convert channel ADC B10: For the voltage across the convertor capacitor
    AdcbRegs.ADCSOC0CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns

    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
    AdcbRegs.ADCSOC1CTL.bit.CHSEL           = 0x07;    // Convert channel ADC B7: For sensing the temperature of the inductor copper.
    AdcbRegs.ADCSOC1CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns
//
//    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
//    AdcbRegs.ADCSOC2CTL.bit.CHSEL           = 0x0C;    // Convert channel ADC B12: Frequency Input
//    AdcbRegs.ADCSOC2CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns

//----- SOC (SOC0, SOC1, SOC2) Configuration for ADC B: END -----

//----- SOC (SOC0, SOC1, SOC2) Configuration for ADC C: START -----
//
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
    AdccRegs.ADCSOC0CTL.bit.CHSEL           = 0x04;    // Convert channel ADC C4: For the output of the buck convertor
    AdccRegs.ADCSOC0CTL.bit.ACQPS           = 0x0B;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns
//
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
    AdccRegs.ADCSOC1CTL.bit.CHSEL           = 0x01;    // Convert channel ADC C1: For sensing the temperature of the inductor core.
    AdccRegs.ADCSOC1CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns


//    AdccRegs.ADCSOC2CTL.bit.TRIGSEL         = 0x05;    // Trigger by ePWM1 SOC A
//    AdccRegs.ADCSOC2CTL.bit.CHSEL           = 0x07;    // Convert channel ADC C4: Fan Failure Voltage detection
//    AdccRegs.ADCSOC2CTL.bit.ACQPS           = 0x09;    // Acquisition window (10 SYSCLK Cycles) i.e., 100ns


//----- SOC (SOC0, SOC1, SOC2) Configuration for ADC C: END -----


// ################################################################################################
// ----------------------------   Triggering the ADC Interrupts   ---------------------------------
// ################################################################################################

    //----- Triggering ADC Interrupts: START -----

    AdcaRegs.ADCINTSOCSEL1.bit.SOC0         = 0;                // No ADCINT will trigger SOC0
    AdcaRegs.ADCINTSOCSEL1.bit.SOC1         = 0;                // No ADCINT will trigger SOC1
    AdcaRegs.ADCINTSOCSEL1.bit.SOC2         = 0;                // No ADCINT will trigger SOC2
    AdcaRegs.ADCINTSOCSEL1.bit.SOC3         = 0;                // No ADCINT will trigger SOC2

    AdcbRegs.ADCINTSOCSEL1.bit.SOC0         = 0;                // No ADCINT will trigger SOC0
    AdcbRegs.ADCINTSOCSEL1.bit.SOC1         = 0;                // No ADCINT will trigger SOC1
    AdcbRegs.ADCINTSOCSEL1.bit.SOC2         = 0;                // No ADCINT will trigger SOC2

    AdccRegs.ADCINTSOCSEL1.bit.SOC0         = 0;                // No ADCINT will trigger SOC0
    AdccRegs.ADCINTSOCSEL1.bit.SOC1         = 0;                // No ADCINT will trigger SOC1
    AdccRegs.ADCINTSOCSEL1.bit.SOC2         = 0;                // No ADCINT will trigger SOC2

    //----- Triggering ADC Interrupts: END -----

// ################################################################################################
// --------------------------------   SOC Priority Selection   ------------------------------------
// ################################################################################################

    //----- SOC Priority Selection for ADC SOC Channels: START -----

    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY   = 0;  // SOC priority mode
    AdcbRegs.ADCSOCPRICTL.bit.SOCPRIORITY   = 0;  // SOC priority mode
    AdccRegs.ADCSOCPRICTL.bit.SOCPRIORITY   = 0;  // SOC priority mode

    //----- SOC Priority Selection for ADC SOC Channels: END -----

// ################################################################################################
// ------------------------------   ADC Interrupt Configuration   ---------------------------------
// ################################################################################################

    //----- Configuring ADC A Interrupt-1 and disable ADC B and ADC C interrupts  : START -----

     AdcaRegs.ADCINTSEL1N2.bit.INT1CONT      = 1;     // Interrupt pulses
     AdcaRegs.ADCINTSEL1N2.bit.INT1E         = 1;     // ADC A interrupt enable
     AdcaRegs.ADCINTSEL1N2.bit.INT1SEL       = 0;     // EOC0 triggers the interrupt

    //----- Configuring ADC A Interrupt-1 and disable ADC C interrupts  : END -----
    //----- Initiating the CMPSS module for the tripping operations---------------//

// ################################################################################################
// ----------------------------------------   Power UP the ADCs --------------------------------
// ################################################################################################

      //******** Powering Up ADCs : START *******

      AdcaRegs.ADCCTL1.bit.ADCPWDNZ           = 1;          // Power up the ADC A
      AdcbRegs.ADCCTL1.bit.ADCPWDNZ           = 1;          // Power up the ADC B
      AdccRegs.ADCCTL1.bit.ADCPWDNZ           = 1;          // Power up the ADC C

      //******** Powering Up ADCs : END *******

      //******** Waiting Period after Power-Up the ADCs : START *******

      DELAY_US(1000);                              // Wait 1 ms after power-up before using the ADC

      //******** Waiting Period after Power-Up the ADCs : END *******

        asm(" EDIS");                               // Disable EALLOW protected register access

}
//
// End of File
//


