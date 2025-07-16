
/**********************************************************************
Code   : Main C file for Variable Frequency Drive (VFD) for Indian Railways
Author : Varun Sharma
**********************************************************************/
#include <AlternatorBatteryChargerF280033.h>         // Main include file
#include "math.h"

//##########################################################################################
//##################################My Global Variables#####################################
//##########################################################################################

//+++++++++++++++++Start of the global variables++++++++++++++++++++
//List of the custom characters
unsigned char TimerShape[8] = {
      0b11111,
      0b01110,
      0b01110,
      0b00100,
      0b00100,
      0b01110,
      0b01110,
      0b11111,
};

unsigned char DarkCell[8] = {
    0b00110,  //   XX
    0b01001,  //  X  X
    0b01001,  //  X  X
    0b00110,  //   XX
    0b00000,  //
    0b00000,  //
    0b00000,  //
    0b00000   //
};
//Testing variables
Uint16 row = 1;
Uint16 col = 1;

//Start of the structures declaraions
SUM_INST_VALS suminstvals = {
    .sum_inst_ntcheatsinktemp = 0.0f,
    .sum_inst_ntcindcopptemp = 0.0f,
    .sum_inst_ntcindcoretemp = 0.0f,
    .sum_currsens = 0.0f,
    .sum_dclink = 0.0f,
    .sum_outvolt = 0.0f,
};

MAX_MIN_REF_VALS refValsStartState = {
     .altOutMax = 240.0f,
     .altOutMin = 130.0f,
     .buckOutMin = 0.0f,
     .buckOutMax = 150.0f,
     .currRefMax = 7.0f,
     .currRefMin = 0.0f,
};

MAX_MIN_REF_VALS refValsActState = {
     .altOutMax = 260.0f,
     .altOutMin = 120.0f,
     .buckOutMin = 0.0f,
     .buckOutMax = 150.0f,
     .currRefMax = 7.0f,
     .currRefMin = 0.0f,
};

TRIPSIGNAS tripsignals = {
//     Initially all the conditions with false to make sure the trip command is not executed wihout the proper sensing.
     .hvovervoltage = 0,
     .hvundervoltage = 0,
     .outovervoltage = 0,
     .overcurrent = 0,
};

CLOSED_LOOP_VARS closedloopmodelcurrloop = {
       .currerr = 0.0f,
       .preverr = 0.0f,
       .integralsum = 0.0f,
       .ref = 2.0f,    // We are setting the reference current to be two amperes.
       .ki = 60.0f,
       .kp = 0.001f,
       .ki_100_times_slower = 0.6f,
       .kp_100_times_slower = 0.00001f,
       .ki_5_times_faster = 300.0f,
       .kp_5_times_faster = 0.005f,
       .sampletime = 0.00003f,      // The sampling time is to be kept at 30microseconds
       .uppersat = 4.0f,
       .lowersat = 0.5f,
};

CLOSED_LOOP_VARS closedloopmodelvoltloop = {
       .currerr = 0.0f,
       .preverr = 0.0f,
       .integralsum = 0.0f,
       .ref = 122.0f,    // We are setting the reference voltage to be 12.0
       .ki = 30.0f,
       .kp = 0.01f,
       .ki_100_times_slower = 0.2f,
       .kp_100_times_slower = 0.00008f,
       .ki_5_times_faster = 100.0f,
       .kp_5_times_faster = 0.04f,
       .sampletime = 0.00003f,      // The sampling time is to be kept at 30microseconds
       .lowersat = 10.0f,
       .uppersat = 50.0f,
};

ALT_BATT_PARAMS multipliers = {
       .currsens = 0.059325f,
       .dcLink = 0.002247191f,
       .outputvolt = 0.00408163f,
       .heatsinkvoltsens = 0.58928571f,
       .inductorcoppvoltsens = 0.58928571f,
       .inductorcorevoltsens = 0.58928571f,
};
ALT_BATT_PARAMS offsets = {
       .currsens = 1.465f,
       .dcLink = 0.0f,
       .outputvolt = 0.0f,
       .heatsinkvoltsens = 0.0f,
       .inductorcoppvoltsens = 0.0f,
       .inductorcorevoltsens = 0.0f,
};

DELAY_CTR delayCtrs;

FAULT_TYPE faultype = {
       .overcurrent = 0,
       .overoutvoltage = 0,
       .undervoltage = 0,
       .overvoltage = 0,
       .hardwareovercurrtrip = 0,
};

ALT_BATT_PARAMS sensedvalues;
ALT_BATT_PARAMS actualvalues;
AVG_VALS avgvalues;
NTC_RES ntcresvals;
CORE_TEMP_INST_VALS insttempvals;

//End of the structures declarations

//Data arrays used
char txData[DATA_LENGTH];                   //For storing the data to be transmitted for the UART transmission
char rxData[DATA_LENGTH];                   //For receiving the data received through the UART communication
char showData[DATA_LENGTH];                 //For the data to be display and numerical values converted into the strings

//Uart sending and receiving indexes
volatile Uint16 dataIdTxUart = 0;
volatile Uint16 dataIdRxUart = 0;

//Counters
volatile Uint32 UartDataCounter = 0;        //Uart data counter
volatile Uint16 avg_vals_counter = 0;       //20ms averaging counter for the average values calculations
volatile Uint32 getNewState = 0;            //For changing the state on the lcd display, counting for 20ms for debouncing of switch
volatile Uint16 updateAfter20ms_Ctr = 0;              //For making sure that the display is updated
                                            //after 20ms only not before that to make
                                            //sure that lcd has always 1.6ms min time
volatile Uint16 UartDataSend = 0;           //For making sure that UART messages are transmitted after a certain period of time
volatile Uint32 trans_counter = 0;          //For counting the time for precise transition from
                                            //one state to the other in the state machine

//Currstate for keeping the track of the current state of the charger
//Prevstate for making sure that state change leads to clear LCD screen once

statemachine currstate = volt_lim_mode;
statemachine prevstate;

float avgCurrOffset = 0.0f;                    //For finding the offset of the current senser
float duty = 0.0f;                             //For keeping the track of the duty cycle
float beta = 3977.0f;                          //For the current sensor beta value, used to
                                               //find the actual temperature from sensed voltages

//LCD Declared logic variables
volatile Uint16 shiftWhatToShow= 0;            //It is used to switch state on the LCD on clicking on  the switch
Uint16 whatToShow = 0;                         //Keep track of the current state to show on the lcd
float outvolt_max = 55.0;                      //For fault logic keeping the track of the max battery voltage
float outvolt_min = 11.0;                      //For fault logic keeping the track of the min battery voltage

//For updating the lcd display on find any fault
Uint16 faultDetected = 0;
Uint16 updateFaultOnceOnly = 0;
Uint16 hit_ref_buck_volt_first_time = 0;
Uint16 hit_max_buck_theshold = 0;
float voltage_margin = 15.0f;
float upper_thres_voltage_margin = 25.0f;
//+++++++++++++++++++++End of the global variables+++++++++++++++++++++

void main(void)
{
    delayCtrs.fivesecdelaycnt = (Uint32)(5.0f/(closedloopmodelvoltloop.sampletime));
    delayCtrs.onesecdelaycnt = (Uint32)(1.0f/(closedloopmodelvoltloop.sampletime));
    delayCtrs.twenty_ms_delay = (Uint32)(0.02f/(closedloopmodelvoltloop.sampletime));
    delayCtrs.twosecdelaycnt = (Uint32)(2.0f/(closedloopmodelvoltloop.sampletime));
    prevstate = currstate;
////--- CPU Initialization
    InitSysCtrl();                      // Initialize the CPU (FILE: f28002x_sysCtrl.c)

    InitGpio();                         // Initialize the shared GPIO pins (FILE: f28002x_gpio.c)

    DINT;

    InitPieCtrl();                      // Initialize and enable the PIE (FILE: PieCtrl.c)

// Initialize the Flash
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    InitFlash();


// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

    InitPieVectTable();

// Map ISR functions

    EALLOW;
    PieVectTable.ADCA1_INT = &ADCA1_ISR;     // Function for ADCA interrupt 1
//    PieVectTable.XINT1_INT = &XINT1_ISR;     // Function for Switch interrupt
//    PieVectTable.XINT2_INT = &XINT2_ISR;     // Function for Switch interrupt
//    PieVectTable.XINT3_INT = &XINT3_ISR;     // Function for Switch interrupt
//    PieVectTable.XINT4_INT = &XINT4_ISR;     // Function for Switch interrupt
    EDIS;

//    Delay_ms(100);

//--- Peripheral Initialization
       initADC();                                      // Initialize the ADC-A (FILE: Adc.c)
       InitCmpss();
       InitEPwm();                                     // Initialize the EPwm (FILE: EPwm.c)
       InitDaca();
//       SCIA_init();
       EALLOW;
//    Enabling the CPU interrupts
       IER |= M_INT1;                                  // Enable INT1 in IER to enable PIE group 1
       IER |= M_INT12;

//     Enable PIE interrupt
       PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
//       PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
//       PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
//       PieCtrlRegs.PIEIER12.bit.INTx1 = 1;
//       PieCtrlRegs.PIEIER12.bit.INTx2 = 1;

//     Acknowledgement of the flags to allow the interrupts
       PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Must acknowledge the PIE group
//       PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;         // Must acknowledge the PIE group

//       For the logic of the Xint interrupts
       InitSwitchesXBar();
       EDIS;

//--- Enable global interrupts
       EINT;           // Enable Global interrupt INTM
       ERTM;           // Enable Global realtime interrupt DBGM

//       Some LCD configurations
       LcdInit();                // Initialize LCD

       EALLOW;
       PieVectTable.SCIA_TX_INT = &SCIA_TX_ISR;  // ISR mapping
       PieVectTable.SCIA_RX_INT = &SCIA_RX_ISR;
       EDIS;

       Lcd_CreateCustomChar(0, TimerShape);      // Create emoji at location 0
       Lcd_CreateCustomChar(1, DarkCell);   // Create emoji at location 0
//       disablehardwaretrip;

//       Initial filling of the FIFO buffer for the first transmission
//       for (; dataIdx<16 && dataIdx<DATA_LENGTH; dataIdx++){
//           SciaRegs.SCITXBUF.bit.TXDT = txData[dataIdx];
//       }
       closedloopmodelvoltloop.kp_100_times_slower = closedloopmodelvoltloop.kp/100.0f;
       closedloopmodelvoltloop.ki_100_times_slower = closedloopmodelvoltloop.ki/100.0f;
       closedloopmodelvoltloop.kp_5_times_faster = closedloopmodelvoltloop.kp*5.0f;
       closedloopmodelvoltloop.ki_5_times_faster = closedloopmodelvoltloop.ki*5.0f;

       while (1){
//           if (UartDataSend == 1){
//               SCIAWrite(txData);
//               UartDataSend = 0;
//           }
           if ((faultype.hardwareovercurrtrip==1) && (updateFaultOnceOnly<=1)){                        //Variable for recognising the hardware tripping
               Lcd_Cmd(0x01);                                             //Clear Screen
               Lcd_out(1, 1, "Hardware Trip!");
               updateFaultOnceOnly++;
           }//End of the trip logic

           if ((shiftWhatToShow == 1) && (faultype.hardwareovercurrtrip == 0)){                // In this condition we can change whether
                                                                              // to see voltage, current or temperature on the LCD display
               Lcd_Cmd(0x01);                                                 // Clearing the display before changing the display screen
               whatToShow = (whatToShow+1)%8;                                 // Actual state transition ctrl+clk to see the rest of states.
               shiftWhatToShow = 0;                                           // This variable activates only on enter pushed down, clearing it.
           }//End of the shifting LCD logic

           if ((updateAfter20ms_Ctr >= delayCtrs.twenty_ms_delay)){           // In controls the frequency of update of the current selected state.
                                                                              // for now we have selected to update after 20 ms using counter
               updateAfter20ms_Ctr=0;                                         // This variable is for updating the LCD display.
               if (faultDetected == 0  && (faultype.hardwareovercurrtrip == 0)){                                       // Update is activated only when there is not fault on the
                                                                              // pcb or other wise there will be collision for the usage of the
                   dispParamsOnLED();                                         // Actual function that is used for the updating lcd
                   updateFaultOnceOnly=0;                                     // This variable makes sure that display screen
                                                                              // command is not activated more frequently than required
               }

               if ((faultDetected == 1) && (updateFaultOnceOnly<=1) && (faultype.hardwareovercurrtrip == 0)){          // If the fault is detected we can display the cause of
                                                                                                                      // the fault rather than normal logic displayed
                   ledCtrlFault();
                   updateFaultOnceOnly++;
               }
           }
       }
}
