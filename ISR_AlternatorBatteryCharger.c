//###########################################################################
//Author: Varun Sharma
//ISRs used: ADC1_ISR, SCI_RX ISR, XINT1, XINT2, XINT3, XINT4, SCI_TX(for testing only)
//###########################################################################

//
// Included Files
//
#include <AlternatorBatteryChargerF280033.h>                        // Main include file
#include "f28003x_device.h"       // f28003x Header File Include File
#include "f28003x_examples.h"     // f28003x Examples Include File
#include <math.h>
//
// CPU Timer 1 Interrupt
//
interrupt void TIMER1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// CPU Timer 2 Interrupt
//
interrupt void TIMER2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// Datalogging Interrupt
//
interrupt void DATALOG_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// RTOS Interrupt from ERAD
//
interrupt void RTOS_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// Emulation Interrupt
//
interrupt void EMU_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// Non-Maskable Interrupt
//
interrupt void NMI_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// Illegal Operation Trap
//
interrupt void ILLEGAL_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 1
//
interrupt void USER1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 2
//
interrupt void USER2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 3
//
interrupt void USER3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 4
//
interrupt void USER4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 5
//
interrupt void USER5_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 6
//
interrupt void USER6_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 7
//
interrupt void USER7_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 8
//
interrupt void USER8_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 9
//
interrupt void USER9_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 10
//
interrupt void USER10_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 11
//
interrupt void USER11_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// User Defined Trap 12
//
interrupt void USER12_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 1.1 - ADCA Interrupt 1
//

//static inline __attribute__((always_inline)) float PIControl(CLOSED_LOOP_VARS* loopCtrl, float error){
//    float ctrlParam = 0.0f;
//    loopCtrl->currerr = error;
//    loopCtrl->integralsum = loopCtrl->integralsum + ((loopCtrl->currerr + loopCtrl->preverr)/(2.0f))*(loopCtrl->sampletime);
//    loopCtrl->preverr = loopCtrl->currerr;
//    ctrlParam  = (loopCtrl->kp * loopCtrl->currerr) + (loopCtrl->ki * loopCtrl->integralsum);
//
//    ctrlParam = (ctrlParma >= loopCtrl->uppersat) ? loopCtrl->uppersat : ctrlParam;
//    ctrlParam = (ctrlParma <= loopCtrl->lowersat) ? loopCtrl->lowersat : ctrlParam;
//
//    return ctrlParam;
//}

interrupt void ADCA1_ISR(void)
{
//    For the LCD logic
    updateAfter20ms_Ctr++;
    if (GpioDataRegs.GPADAT.bit.GPIO22 == 0){
        getNewState++;
    }
//    Logic for shifting the state
    if (GpioDataRegs.GPADAT.bit.GPIO22 == 1){
        if (getNewState >= (delayCtrs.twenty_ms_delay)){
            getNewState = 0;
            shiftWhatToShow = 1;
        }
        getNewState = 0;
    }

//    Getting the sensed values in the voltages
    sensedvalues.currsens = (AdcaResultRegs.ADCRESULT0)*(3.3f/4096.0f);
    sensedvalues.highvolt = (AdcbResultRegs.ADCRESULT0)*(3.3f/4096.0f);
    sensedvalues.outputvolt = (AdccResultRegs.ADCRESULT0)*(3.3f/4096.0f);
    sensedvalues.heatsinkvoltsens = (AdcaResultRegs.ADCRESULT1)*(3.3f/4096.0f);
    sensedvalues.inductorcoppvoltsens = (AdcbResultRegs.ADCRESULT1)*(3.3f/4096.0f);
    sensedvalues.inductorcorevoltsens = (AdccResultRegs.ADCRESULT1)*(3.3f/4096.0f);

//    Taking the effect of the offsets and multipliers
    actualvalues.currsens = (sensedvalues.currsens-offsets.currsens)/(multipliers.currsens);
    actualvalues.highvolt = (sensedvalues.highvolt-offsets.highvolt)/(multipliers.highvolt);
    actualvalues.outputvolt = (sensedvalues.outputvolt-offsets.outputvolt)/(multipliers.outputvolt);

//    For getting the resistances values
    ntcresvals.ntcheatsinkres = (9435.668f/(sensedvalues.heatsinkvoltsens)) - 8201.4388f;
    ntcresvals.ntcindcoppres = (9435.668f/(sensedvalues.inductorcoppvoltsens)) - 8201.4388f;
    ntcresvals.ntcindcoreres = (9435.668f/(sensedvalues.inductorcorevoltsens)) - 8201.4388f;

//    For finding the temperature at every sample of the resistance found or calculated
    insttempvals.inst_ntcheatsinktemp = (1.0f/((1/298.0f) + ((1/beta) * logf((ntcresvals.ntcheatsinkres)/10000.0)))) - 273.0f; // in degree celsius
    insttempvals.inst_ntcindcopptemp = (1.0f/((1/298.0f) + ((1/beta) * logf((ntcresvals.ntcindcoppres)/10000.0)))) - 273.0f; // in degree celsius
    insttempvals.inst_ntcindcoretemp = (1.0f/((1/298.0f) + ((1/beta) * logf((ntcresvals.ntcindcoreres)/10000.0)))) - 273.0f; // in degree celsius

    suminstvals.sum_inst_ntcheatsinktemp += insttempvals.inst_ntcheatsinktemp;
    suminstvals.sum_inst_ntcindcopptemp += insttempvals.inst_ntcindcopptemp;
    suminstvals.sum_inst_ntcindcoretemp += insttempvals.inst_ntcindcoretemp;
    suminstvals.sum_currsens += actualvalues.currsens;
    suminstvals.sum_dclink += actualvalues.highvolt;
    suminstvals.sum_outvolt += actualvalues.outputvolt;

    avg_vals_counter++;
    if (avg_vals_counter >= delayCtrs.twenty_ms_delay){
        avgvalues.avgheatsinktemp = suminstvals.sum_inst_ntcheatsinktemp/(delayCtrs.twenty_ms_delay);
        suminstvals.sum_inst_ntcheatsinktemp = 0.0f;

        avgvalues.avgindcopptemp = suminstvals.sum_inst_ntcindcopptemp/(delayCtrs.twenty_ms_delay);
        suminstvals.sum_inst_ntcindcopptemp = 0.0f;

        avgvalues.avgindcoretemp = suminstvals.sum_inst_ntcindcoretemp/(delayCtrs.twenty_ms_delay);
        suminstvals.sum_inst_ntcindcoretemp = 0.0f;

        avgvalues.avgcurrsens = suminstvals.sum_currsens/(delayCtrs.twenty_ms_delay);
        suminstvals.sum_currsens = 0.0f;

        avgvalues.avgdclinkvoltage = suminstvals.sum_dclink/(delayCtrs.twenty_ms_delay);
        suminstvals.sum_dclink = 0.0f;

        avgvalues.avgoutvoltage = suminstvals.sum_outvolt/(delayCtrs.twenty_ms_delay);
        suminstvals.sum_outvolt = 0.0f;

        avg_vals_counter = 0;
    }

//    End of the code for the buck convertor closed loop
//#######################Creating the state machine for the alternating battery charger##################
//
    switch (currstate){
        case volt_lim_mode:{
//            Initialising all the LEDs to to default state of the voltage limited mode
            gridPresent_led_turnoff;            //Initialising all the leds to turn off
            fault_led_turnoff;
            invertorOn_led_turnoff;
            pvPresent_led_turnoff;

            trans_counter=0;                    //Initialising the counter for the transition between the states

//            Setting the duty cycle of the EPwm1 to zero
            duty = 0.0f;
            Uint16 cmpaval = (Uint16)((Tbprd)*(1-duty));
            EPwm1Regs.CMPA.bit.CMPA = cmpaval;

//            Checking whether getting proper voltage at the input side
            if (StartupCond){
                gridPresent_led_turnon;                 // Grid present led is turned on
                currstate = one_sec_delay_mode;
            }
            break;  // Capacitor voltage is well within the limits.
        }

        case one_sec_delay_mode:{
            if (!StartupCond){
                gridPresent_led_turnoff;                                    // Grid is not present as voltage is not in limit
                currstate = volt_lim_mode;                                  // Drawing back to the voltage limited mode.
                break;
            }
            trans_counter++;
            if (trans_counter >= delayCtrs.onesecdelaycnt){            // ADC triggers after every 30 micro seconds so we need about 33333 iteration to cover 1s
                trans_counter = 0;
                currstate = two_sec_off_avg_mode;
            }
            break;  // Got the system stable for 1 sec.
        }
        case two_sec_off_avg_mode:{
            if (!StartupCond){
                gridPresent_led_turnoff;                                    // Grid is not present as voltage is not in limit
                currstate = volt_lim_mode;                                  // Drawing back to the voltage limited mode.
                break;
            }
            trans_counter++;
            if (trans_counter < delayCtrs.twosecdelaycnt){
                avgCurrOffset = avgCurrOffset*(((float)trans_counter-1)/(float)trans_counter) + sensedvalues.currsens*(1.0f/((float)trans_counter));  // Doing the online average to save the space for storing the large values.
            }
            if (trans_counter >= delayCtrs.twosecdelaycnt){
                trans_counter=0;
                offsets.currsens = avgCurrOffset;
                currstate = activelogic;
            }
            break;  // End of the active state logic.
        }
        case activelogic:{
//          Checking out put voltage in limit
            pvPresent_led_turnon;
            if (!output_volt_in_lim){
                pvPresent_led_turnoff;
            }

            if (ActiveTripCond){                            //      For detecting any fault present in the supply
                                                            //      voltage or the output current or the battery voltage
                if (overcurrentfault){                      //      For checking for overcurrent fault
                    faultype.overcurrent = 1;               //      This variable turns on on overcurrent detection
                }
                else if (undervoltgefault){                 //      This condition is for the undervoltage detection
                    faultype.undervoltage = 1;              //      Variable is activated same for the rest two
                }
                else if (overvoltagefault){
                    faultype.overvoltage = 1;
                }
                else if (overoutvoltagefault){
                    faultype.overoutvoltage = 1;
                };

                invertorOn_led_turnoff;
                currstate = triplogic;
                break;
            }
            invertorOn_led_turnon;

        //    EPwm given to the buck convertor for the closed loop control.

//#####################Code for the closed loop control of the outer loop start########################
            closedloopmodelvoltloop.currerr = closedloopmodelvoltloop.ref - actualvalues.outputvolt;
            closedloopmodelvoltloop.integralsum = closedloopmodelvoltloop.integralsum + ((closedloopmodelvoltloop.currerr + closedloopmodelvoltloop.preverr)/(2.0f))*(closedloopmodelvoltloop.sampletime);
            closedloopmodelvoltloop.preverr = closedloopmodelvoltloop.currerr;
            closedloopmodelcurrloop.ref  = (closedloopmodelvoltloop.kp * closedloopmodelvoltloop.currerr) + (closedloopmodelvoltloop.ki * closedloopmodelvoltloop.integralsum);

            if (closedloopmodelcurrloop.ref > closedloopmodelcurrloop.uppersat){
                closedloopmodelcurrloop.ref = closedloopmodelcurrloop.uppersat;
            }
            if (closedloopmodelcurrloop.ref < closedloopmodelcurrloop.lowersat){
                closedloopmodelcurrloop.ref = closedloopmodelcurrloop.lowersat;
            }
//#####################Code for the closed loop control of the outer loop end#########################

//######################Code for the current control of the inner loop################################
            closedloopmodelcurrloop.currerr = closedloopmodelcurrloop.ref - actualvalues.currsens;
            closedloopmodelcurrloop.integralsum = closedloopmodelcurrloop.integralsum + ((closedloopmodelcurrloop.currerr + closedloopmodelcurrloop.preverr)/(2.0f))*(closedloopmodelcurrloop.sampletime);
            closedloopmodelcurrloop.preverr = closedloopmodelcurrloop.currerr;
            duty = (closedloopmodelcurrloop.kp * closedloopmodelcurrloop.currerr) + (closedloopmodelcurrloop.ki * closedloopmodelcurrloop.integralsum);

            //    Updating the duty cycle
            duty = (duty > 1.0f) ? 1.0f : duty;
            duty = (duty < 0.0f) ? 0.0f : duty;

            Uint16 cmpaval = (Uint16)((Tbprd)*(1-duty));
            EPwm1Regs.CMPA.bit.CMPA = cmpaval;
            break;
        }                                                           //End of the active state logic

        case triplogic:{
            if (!output_volt_in_lim){                               //Regularly monitoring whether the battery
                pvPresent_led_turnoff;                              //is maintained at the desired voltage range or not
            }else{
                pvPresent_led_turnon;                               //Even after tripping if the battery voltage stays in limit
            };                                                      //we will keep the pv present led turned on, as it depends on
                                                                    //volage across the terminals of the battery, and may discharge
                                                                    //with time

            invertorOn_led_turnoff;                                 //Since we are in the trip state the convertor is off
            fault_led_turnon;                                       //Fault is detected and led is tuned on to reflect the same.
            faultDetected = 1;                                      //Indicator of the fault detection

//            Reinitialising the PI controller variables
            closedloopmodelcurrloop.integralsum = 0.0f;             /*      \                                                                                      */
            closedloopmodelvoltloop.integralsum = 0.0f;             /*       \                                                                                     */
            closedloopmodelvoltloop.preverr = 0.0f;                 /*        \    These lines are used for the reinitialising of the pi variables                  */
            closedloopmodelcurrloop.preverr = 0.0f;                 /*        /    We need to make  sure that all the PI controller variables are initialised      */
            closedloopmodelcurrloop.currerr = 0.0f;                 /*       /                                                                                     */
            closedloopmodelvoltloop.currerr = 0.0f;                 /*      /                                                                                      */
            closedloopmodelcurrloop.ref = 2.0f;                     /*     /                                                                                       */

            trans_counter=0;                                        /*              Reinitialising the transition counter */

            duty = 0.0f;                                            /*      \                                                                                         */
            Uint16 cmpaval = (Uint16)((Tbprd)*(1-duty));            /*       |   Reinitialising the buck convertor                                                                                       */
            EPwm1Regs.CMPA.bit.CMPA = cmpaval;                      /*      /                                                                                         */

//            For the five second delay not doing anything
            if (trans_counter < delayCtrs.fivesecdelaycnt){     // Dont do anything for 5secs
                trans_counter++;
                break;
            }
            if (trans_counter >= delayCtrs.fivesecdelaycnt){
                if (RestartFromTripCond){                           /*This condition is based on the startup condition and not on active state hv max and min voltages */
                    trans_counter = 0;
                    faultDetected = 0;                              //Clearing the fault indicator for removing the fault
                    fault_led_turnoff;
                    currstate = volt_lim_mode;
                }
            }
            break;
        }
    }
//#########################End of the state machine##################################################

//#####################For the uart message transmission##########################
//    UartDataCounter++;
//    if (UartDataCounter >= 200000){        //It means after every 5 seconds.
//        UartDataSend = 1;       //It means that the data can be sent via the uart in the main
//        UartDataCounter = 0;
//        publishData(txData);
//    }
//##################End of the uart message transmission#######################

//    Ending the state machine for the alternating battery charger
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Must acknowledge the PIE group

    //--- Manage the ADC registers
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear ADCINT1 flag
}

//
// 1.2 - ADCB Interrupt 1
//
interrupt void ADCB1_ISR(void)
{

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Must acknowledge the PIE group

    //--- Manage the ADC registers
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear ADCINT1 flag
}

//
// 1.3 - ADCC Interrupt 1
//
interrupt void ADCC1_ISR(void)
{

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Must acknowledge the PIE group

    //--- Manage the ADC registers
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear ADCINT1 flag
}

//
// 1.4 - XINT1 Interrupt
// Enter
//    col++;
//    Lcd_out(row, col, "");
//    LcdData(1);

interrupt void XINT1_ISR(void)
{
    ledCtrl(toggle4);
//    if (col+1 > 16){
//        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//        return;
//    }
//    Lcd_Cmd(0x01);
//    addInteger(avgtempvalues.avgheatsinktemp, showData);
//    Lcd_out(1, 1, "Heat Sink Temp: ");
//    Lcd_out(2, 1, showData);
//    Lcd_out(2, 7, "");
//    LcdData(1);
//    Lcd_out(2, 8, "C");
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// 1.5 - XINT2 Interrupt
// Down
interrupt void XINT2_ISR(void)
{
    ledCtrl(toggle2);
//    if (row+1 > 2){
//        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//        return;
//    }
//    Lcd_Cmd(0x01);
//    addInteger(avgtempvalues.avgindcopptemp, showData);
//    Lcd_out(1, 1, "Ind Copper Temp:");
//    Lcd_out(2, 1, showData);
//    Lcd_out(2, 7, "");
//    LcdData(1);
//    Lcd_out(2, 8, "C");
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// 1.7 - Timer 0 Interrupt
//
interrupt void TIMER0_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 1.8 - Standby and Halt Wakeup Interrupt
//
interrupt void WAKE_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.1 - ePWM1 Trip Zone Interrupt
//
interrupt void EPWM1_TZ_ISR(void)
{

    //MainContactorOFF = 1;
//    state = tripState;
//    faultPresent = 1;
//
//    if (softwareTrip == 0)
//    {
//        // Finding out the reason for hardware tripping
//        if (Cmpss1Regs.COMPSTS.bit.COMPHLATCH == 1)       // phase a high
//        {
//         faultCode = 20;
//        }
//        else if (Cmpss1Regs.COMPSTS.bit.COMPLLATCH == 1)   // phase a low
//        {
//            faultCode = 21;
//        }
//        else if (Cmpss4Regs.COMPSTS.bit.COMPHLATCH == 1)   // phase b high
//        {
//            faultCode = 22;
//        }
//        else if (Cmpss4Regs.COMPSTS.bit.COMPLLATCH == 1)   // phase b low
//        {
//            faultCode = 23;
//        }
//        else if (Cmpss2Regs.COMPSTS.bit.COMPHLATCH == 1)   // phase c high
//        {
//            faultCode = 24;
//        }
//        else if (Cmpss2Regs.COMPSTS.bit.COMPLLATCH == 1)   // phase c low
//        {
//            faultCode = 25;
//        }
//        else                                               // spurious
//        {
//            faultCode = 10;
//        }
//
//
//    }
    Lcd_Cmd(0x01);
    Lcd_out(1, 1, "SYSTEM TRIP");
    Lcd_out(2, 1, "BABE RESTART!!");

    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

//
// 2.2 - ePWM2 Trip Zone Interrupt
//
interrupt void EPWM2_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.3 - ePWM3 Trip Zone Interrupt
//
interrupt void EPWM3_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.4 - ePWM4 Trip Zone Interrupt
//
interrupt void EPWM4_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.5 - ePWM5 Trip Zone Interrupt
//
interrupt void EPWM5_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.6 - ePWM6 Trip Zone Interrupt
//
interrupt void EPWM6_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.7 - ePWM7 Trip Zone Interrupt
//
interrupt void EPWM7_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 2.8 - ePWM8 Trip Zone Interrupt
//
interrupt void EPWM8_TZ_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.1 - ePWM1 Interrupt
//
interrupt void EPWM1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.2 - ePWM2 Interrupt
//
interrupt void EPWM2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.3 - ePWM3 Interrupt
//
interrupt void EPWM3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.4 - ePWM4 Interrupt
//
interrupt void EPWM4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.5 - ePWM5 Interrupt
//
interrupt void EPWM5_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.6 - ePWM6 Interrupt
//
interrupt void EPWM6_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.7 - ePWM7 Interrupt
//
interrupt void EPWM7_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 3.8 - ePWM8 Interrupt
//
interrupt void EPWM8_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 4.1 - eCAP1 Interrupt
//
interrupt void ECAP1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 4.2 - eCAP2 Interrupt
//
interrupt void ECAP2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 4.3 - eCAP3 Interrupt
//
interrupt void ECAP3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.1 - eQEP1 Interrupt
//
interrupt void EQEP1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.2 - eQEP2 Interrupt
//
interrupt void EQEP2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.5 - CLB1 (Reconfigurable Logic) Interrupt
//
interrupt void CLB1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.6 - CLB2 (Reconfigurable Logic) Interrupt
//
interrupt void CLB2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.7 - CLB3 (Reconfigurable Logic) Interrupt
//
interrupt void CLB3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.8 - CLB4 (Reconfigurable Logic) Interrupt
//
interrupt void CLB4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.1 - SPIA Receive Interrupt
//
interrupt void SPIA_RX_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.2 - SPIA Transmit Interrupt
//
interrupt void SPIA_TX_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.3 - SPIB Receive Interrupt
//
interrupt void SPIB_RX_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.4 - SPIB Transmit Interrupt
//
interrupt void SPIB_TX_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.1 - DMA Channel 1 Interrupt
//
interrupt void DMA_CH1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.2 - DMA Channel 2 Interrupt
//
interrupt void DMA_CH2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.3 - DMA Channel 3 Interrupt
//
interrupt void DMA_CH3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.4 - DMA Channel 4 Interrupt
//
interrupt void DMA_CH4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.5 - DMA Channel 5 Interrupt
//
interrupt void DMA_CH5_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.6 - DMA Channel 6 Interrupt
//
interrupt void DMA_CH6_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.1 - I2CA Interrupt 1
//
interrupt void I2CA_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.2 - I2CA Interrupt 2
//
interrupt void I2CA_FIFO_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.3 - I2CB Interrupt 1
//
interrupt void I2CB_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.4 - I2CB Interrupt 2
//
interrupt void I2CB_FIFO_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.1 - SCIA Receive Interrupt

interrupt void SCIA_RX_ISR(void)
{
    if (dataIdRxUart >= DATA_LENGTH){
        dataIdRxUart = 0;
    }

    if (SciaRegs.SCIRXBUF.bit.SAR == '\0'){
        SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
        SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
        return;
    }

    rxData[dataIdRxUart++] = SciaRegs.SCIRXBUF.bit.SAR;

    // Clear interrupt flags
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


//
// 9.2 - SCIA Transmit Interrupt
//
interrupt void SCIA_TX_ISR(void)
{
//    Returning without loading as all the data has been loaded
    if (dataIdTxUart >= DATA_LENGTH){
//        SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
//        PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
//        return;
        dataIdTxUart = 0;
    }

//    Again load the fifo buffer with 16 bytes of data
    SciaRegs.SCITXBUF.bit.TXDT = txData[dataIdTxUart++];
    for (; dataIdTxUart%16 != 0 && dataIdTxUart<DATA_LENGTH; dataIdTxUart++){
        SciaRegs.SCITXBUF.bit.TXDT = txData[dataIdTxUart];
    }
//    Data loaded

//    Clearing interrupt on the peripheral level
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//
// 9.3 - SCIB Receive Interrupt
//
interrupt void SCIB_RX_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.4 - SCIB Transmit Interrupt
//
interrupt void SCIB_TX_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.5 - CANA Interrupt 0
//
interrupt void CANA0_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.6 - CANA Interrupt 1
//
interrupt void CANA1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.1 - ADCA Event Interrupt
//
interrupt void ADCA_EVT_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.2 - ADCA Interrupt 2
//
interrupt void ADCA2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.3 - ADCA Interrupt 3
//
interrupt void ADCA3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.4 - ADCA Interrupt 4
//
interrupt void ADCA4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.5 - ADCB Event Interrupt
//
interrupt void ADCB_EVT_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.6 - ADCB Interrupt 2
//
interrupt void ADCB2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.7 - ADCB Interrupt 3
//
interrupt void ADCB3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.8 - ADCB Interrupt 4
//
interrupt void ADCB4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.1 - CLA1 Interrupt 1
//
interrupt void CLA1_1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.2 - CLA1 Interrupt 2
//
interrupt void CLA1_2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.3 - CLA1 Interrupt 3
//
interrupt void CLA1_3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.4 - CLA1 Interrupt 4
//
interrupt void CLA1_4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.5 - CLA1 Interrupt 5
//
interrupt void CLA1_5_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.6 - CLA1 Interrupt 6
//
interrupt void CLA1_6_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.7 - CLA1 Interrupt 7
//
interrupt void CLA1_7_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 11.8 - CLA1 Interrupt 8
//
interrupt void CLA1_8_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.1 - XINT3 Interrupt
// Back
interrupt void XINT3_ISR(void)
{
    ledCtrl(toggle1);
//    if (col-1 < 1){
//        PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
//        return;
//    }
//    Lcd_Cmd(0x01);
//    addInteger(avgtempvalues.avgindcoretemp, showData);
//    Lcd_out(1, 1, "Ind Core Temp:");
//    Lcd_out(2, 1, showData);
//    Lcd_out(2, 7, "");
//    LcdData(1);
//    Lcd_out(2, 8, "C");
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

//
// 12.2 - XINT4 Interrupt
// Up
interrupt void XINT4_ISR(void)
{
    ledCtrl(toggle3);
//    if (row-1 < 1){
//        PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
//        return;
//    }
//    Lcd_Cmd(0x01);
//    row--;
//    Lcd_out(row, col, "");
//    LcdData(1);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

//
// 12.3 - XINT5 Interrupt
//
interrupt void XINT5_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.4 - MPOST Interrupt
//
interrupt void MPOST_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.5 - Flash Wrapper Operation Done Interrupt
//
interrupt void FMC_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.7 - FPU Overflow Interrupt
//
interrupt void FPU_OVERFLOW_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.8 - FPU Underflow Interrupt
//
interrupt void FPU_UNDERFLOW_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 1.10 - SYS_ERR interrupt
//
interrupt void SYS_ERR_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 4.11 - eCAP3_2 Interrupt
//
interrupt void ECAP3_2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.9 - SDFM1 Interrupt
//
interrupt void SDFM1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.10 - SDFM2 Interrupt
//
interrupt void SDFM2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.13 - SDFM1DR1 Interrupt
//
interrupt void SDFM1DR1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.14 - SDFM1DR2 Interrupt
//
interrupt void SDFM1DR2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.15 - SDFM1DR3 Interrupt
//
interrupt void SDFM1DR3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 5.16 - SDFM1DR4 Interrupt
//
interrupt void SDFM1DR4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.13 - SDFM2DR1 Interrupt
//
interrupt void SDFM2DR1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.14 - SDFM2DR2 Interrupt
//
interrupt void SDFM2DR2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.15 - SDFM2DR3 Interrupt
//
interrupt void SDFM2DR3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 6.16 - SDFM2DR4 Interrupt
//
interrupt void SDFM2DR4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.11 - FSITXA_INT1 Interrupt
//
interrupt void FSITXA1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.12 - FSITXA_INT2 Interrupt
//
interrupt void FSITXA2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.13 - FSIRXA_INT1 Interrupt
//
interrupt void FSIRXA1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.14 - FSIRXA_INT2 Interrupt
//
interrupt void FSIRXA2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 7.16 - DCC0 Interrupt
//
interrupt void DCC0_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.9 - LINA Interrupt0
//
interrupt void LINA_0_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.10 - LINA Interrupt1
//
interrupt void LINA_1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.11 - LINB Interrupt0
//
interrupt void LINB_0_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.12 - LINB Interrupt1
//
interrupt void LINB_1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.13 - PMBUSA Interrupt
//
interrupt void PMBUSA_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 8.16 - DCC1 Interrupt
//
interrupt void DCC1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.9 - MCAN Sub-System Interrupt 0
//
interrupt void MCANA_0_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.10 - MCAN Sub-System Interrupt 1
//
interrupt void MCANA_1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.11 - MCAN Sub-System ECC error Interrupt
//
interrupt void MCANA_ECC_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.12 - MCAN Sub-System wakeup Interrupt
//
interrupt void MCANA_WAKE_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.13 - BGCRC_CPU
//
interrupt void BGCRC_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 9.16 - HICA Interrupt
//
interrupt void HICA_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.9 - ADCC Event Interrupt
//
interrupt void ADCC_EVT_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.10 - ADCC Interrupt 2
//
interrupt void ADCC2_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.11 - ADCC Interrupt 3
//
interrupt void ADCC3_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 10.12 - ADCC Interrupt 4
//
interrupt void ADCC4_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.10 - RAM Correctable Error Interrupt
//
interrupt void RAM_CORR_ERR_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.11 - Flash Correctable Error Interrupt
//
interrupt void FLASH_CORR_ERR_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.12 - RAM Access Violation Interrupt
//
interrupt void RAM_ACC_VIOL_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.13 - AES Secure Interrupt request
//
interrupt void AES_SINTREQUEST_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.14 - BGCRC CLA1
//
interrupt void BGCRC_CLA1_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.15 - CLA Overflow Interrupt
//
interrupt void CLA_OVERFLOW_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// 12.16 - CLA Underflow Interrupt
//
interrupt void CLA_UNDERFLOW_ISR(void)
{
    //
    // Insert ISR Code here
    //

    //
    // To receive more interrupts from this PIE group,
    // acknowledge this interrupt.
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    //

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
    asm ("      ESTOP0");
    for(;;);
}

//
// Catch-all Default ISRs:
//

//
// PIE_RESERVED_ISR - Reserved ISR
//
interrupt void PIE_RESERVED_ISR(void)
{
    asm ("      ESTOP0");
    for(;;);
}

//
// EMPTY_ISR - Only does a return
//
interrupt void EMPTY_ISR(void)
{

}

//
// NOTUSED_ISR - Unused ISR
//
interrupt void NOTUSED_ISR(void)
{
    asm ("      ESTOP0");
    for(;;);
}

//
// End of File
//

