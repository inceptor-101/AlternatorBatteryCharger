
/**********************************************************************
* File: Self Defined.h
  Self Defined Header File

**********************************************************************/
#include "f28003x_device.h"
#ifndef LAB_H
#define LAB_H


//---------------------------------------------------------------------------
// Constant Definitions
//

// Digital Outputs
/*
  GPIO7 - Main Relay
  GPIO29- Soft Charge Relay
  GPIO8 - LED1
  GPIO9 - LED2
  GPIO10- LED3
  GPIO11- LED4
  GPIO28- D7 for LCD display
  GPIO13- D6 for LCD display
  GPIO12- D5 for LCD display
  GPIO33- D4 for LCD display
  GPIO17- RS for LCD display
  GPIO16- EN for LCD display

 */

// Digital Inputs
/*
 GPIO22 - SW1
 GPIO41 - SW2
 GPIO23 - SW3
 GPIO40 - SW4
 GPIO37 - ON/OFF Control
 */
// LCD Pin Definitions and Functions Definitions
#define enablePinON     GpioDataRegs.GPASET.bit.GPIO16
#define rsPinON         GpioDataRegs.GPASET.bit.GPIO17
#define d7ON            GpioDataRegs.GPASET.bit.GPIO28
#define d6ON            GpioDataRegs.GPASET.bit.GPIO13
#define d5ON            GpioDataRegs.GPASET.bit.GPIO12
#define d4ON            GpioDataRegs.GPBSET.bit.GPIO33
#define DATA_LENGTH     1000  // total number of bytes you want to send


#define enablePinOFF   GpioDataRegs.GPACLEAR.bit.GPIO16
#define rsPinOFF       GpioDataRegs.GPACLEAR.bit.GPIO17
#define d7OFF          GpioDataRegs.GPACLEAR.bit.GPIO28
#define d6OFF          GpioDataRegs.GPACLEAR.bit.GPIO13
#define d5OFF          GpioDataRegs.GPACLEAR.bit.GPIO12
#define d4OFF          GpioDataRegs.GPBCLEAR.bit.GPIO33

//#######################################Global Variables###########################################
//+++++++++++For using LED++++++++++++
extern Uint16 stringLen(char* ipStr);
extern void LcdInit(void);
extern void Lcd_Cmd(unsigned char value);
extern void LcdData(unsigned char value);
extern void Lcd_out(Uint16 rowNumber, Uint16 colNumber, char* dataDisplay);
extern void Lcd_CreateCustomChar(Uint16 location, unsigned char* pattern);
extern void InitSwitchesXBar(void);
extern unsigned char TimerShape[8];
extern unsigned char emoji[8];
extern Uint16 row;
extern Uint16 col;
extern volatile Uint16 avg_vals_counter;
extern volatile Uint16 UartDataSend;
extern volatile Uint32 UartDataCounter;
extern void publishData(char* txData);
extern void ledCtrlFault(void);
//+++++++++++End of Led variables+++++++++

//+++++++++++++++++Start of the structue definitions+++++++++++++++++++++
typedef struct{
    float dcLink;
    float currsens;
    float outputvolt;
    float heatsinkvoltsens;
    float inductorcoppvoltsens;
    float inductorcorevoltsens;
}ALT_BATT_PARAMS;

typedef struct{
    float altOutMax;
    float altOutMin;
    float buckOutMax;
    float buckOutMin;
    float currRefMax;
    float currRefMin;
}MAX_MIN_REF_VALS;


enum ledcontrol{
//    From left to right
    turnon1,
    toggle1,     //For led with gpio8
    turnoff1,

    turnon2,
    toggle2,    //For led with gpio9
    turnoff2,

    turnon3,
    toggle3,    //For led with Gpio10
    turnoff3,

    turnon4,
    toggle4,    //For led with gpio11
    turnoff4,
};

typedef enum{
    volt_lim_mode,
    one_sec_delay_mode,
    two_sec_off_avg_mode,
    activelogic,
    triplogic
}statemachine;

typedef enum{
    show_state,
    show_output_volt,
    show_curr,
    show_dc_link_volt,
    show_heat_sink_temp,
    show_ind_copp_temp,
    show_ind_core_temp,
    show_duty_cycle,
}lcdDisplay;

typedef struct{
    Uint16 hvovervoltage : 1;
    Uint16 hvundervoltage : 1;
    Uint16 overcurrent : 1;
    Uint16 outovervoltage : 1;
}TRIPSIGNAS;

typedef struct{
    float avgcurrsens;
    float avgoutvoltage;
    float avgdclinkvoltage;
    float avgheatsinktemp;
    float avgindcopptemp;
    float avgindcoretemp;
}AVG_VALS;

typedef struct{
    float ntcheatsinkres;
    float ntcindcoppres;
    float ntcindcoreres;
}NTC_RES;

typedef struct{
    float inst_ntcheatsinktemp;
    float inst_ntcindcopptemp;
    float inst_ntcindcoretemp;
}CORE_TEMP_INST_VALS;

typedef struct{
    float sum_inst_ntcheatsinktemp;
    float sum_inst_ntcindcopptemp;
    float sum_inst_ntcindcoretemp;
    float sum_currsens;
    float sum_outvolt;
    float sum_dclink;
}SUM_INST_VALS;

typedef struct{
    Uint32 twenty_ms_delay;
    Uint32 onesecdelaycnt;
    Uint32 twosecdelaycnt;
    Uint32 fivesecdelaycnt;
}DELAY_CTR;

typedef struct{
    float kp;
    float ki;
    float kp_100_times_slower;
    float ki_100_times_slower;
    float kp_5_times_faster;
    float ki_5_times_faster;
    float ref;
    float currerr;
    float preverr;
    float integralsum;
    float sampletime;
    float uppersat;
    float lowersat;
}CLOSED_LOOP_VARS;

typedef struct{
    Uint16 hardwareovercurrtrip;
    Uint16 overcurrent;
    Uint16 overvoltage;
    Uint16 undervoltage;
    Uint16 overoutvoltage;
}FAULT_TYPE;
//+++++++++++++++++++End of the structure definitions++++++++++++++++++++++

//+++++++++++++Start of my Global variables declarations++++++++++++++
extern statemachine currstate;
extern statemachine prevstate;
extern float expected;
extern float duty;
extern volatile Uint16 updateAfter20ms_Ctr;
extern float avgCurrOffset;
extern float beta;
extern volatile Uint32 trans_counter;
extern Uint16 whatToShow;
extern ALT_BATT_PARAMS sensedvalues;
extern ALT_BATT_PARAMS actualvalues;
extern ALT_BATT_PARAMS multipliers;
extern ALT_BATT_PARAMS offsets;
extern MAX_MIN_REF_VALS refValsStartState;
extern MAX_MIN_REF_VALS refValsActState;
extern AVG_VALS avgvalues;
extern NTC_RES ntcresvals;
extern CORE_TEMP_INST_VALS insttempvals;
extern TRIPSIGNAS tripsignals;
extern SUM_INST_VALS suminstvals;
extern CLOSED_LOOP_VARS closedloopmodelcurrloop;
extern CLOSED_LOOP_VARS closedloopmodelvoltloop;
extern DELAY_CTR delayCtrs;
extern FAULT_TYPE faultype;
//+++++++++++++++End of my Global variables declarations++++++++++++++++

//++++++++++++Macros definitions start++++++++++++++++
#define PWM_PERIOD_INVERTER 6000;
//++++++++++++Macros definitions end++++++++++++++++++

//---------------------------------------------------------------------------
// Include Standard C Language Header Files
// (Header file <string.h> not supported by CLA compiler)
//
#if !defined(__TMS320C28XX_CLA__)
    #include <string.h>
#endif


//---------------------------------------------------------------------------
// Include any other Header Files
//
//#include "F2837xD_Cla_typedefs.h"        // CLA type definitions
//#include "F2837xD_device.h"              // F2837xD header file peripheral address definitions
//#include "F2837xD_Adc_defines.h"         // ADC definitions
//#include "F2837xD_defaultisr.h"          // ISR definitions
//#include "F2837xD_Pie_defines.h"         // PIE definitions

#include "f28003x_examples.h"    // F28002x Examples Include File
#include "f28003x_device.h"      // f28002x Headerfile Include File


//---------------------------------------------------------------------------
// Function Prototypes
//
extern void Delay_us(Uint32);
extern void AdcSetMode(Uint16, Uint16, Uint16);
extern void CalAdcINL(Uint16);
extern void DelayUs(Uint16);
extern void initADC(void);
extern void InitCla(void);
extern void InitDacb(void);
extern void InitDaca(void);
extern void InitDacc(void);
extern void InitDma(void);
extern void InitECap(void);
extern void InitEPwm(void);
extern void InitFlash(void);
extern void InitGpio(void);
extern void setup1GPIO(void);
extern void setup2GPIO(void);
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);
extern void InitWatchdog(void);
extern void InitCmpss(void);
extern void InitXBars(void);
extern void InitTripCfg(void);
extern void InitTripCfgTrial(void);
extern void SetDBGIER(Uint16);
extern void UserInit(void);
extern void Delay_ms(Uint16);
extern void SCIA_init(void);
extern void SCIAWrite(char* str);
extern void updateduty(void);
extern char* addInteger(float val, char* txData);
extern char txData[DATA_LENGTH];
extern char rxData[DATA_LENGTH];
extern char showData[DATA_LENGTH];;
extern volatile Uint16 fifosts;
extern volatile Uint16 dataIdTxUart;
extern volatile Uint16 dataIdRxUart;
extern volatile Uint32 getNewState;
extern volatile Uint16 shiftWhatToShow;
extern Uint16 hit_ref_buck_volt_first_time;
extern Uint16 hit_max_buck_theshold;
extern float voltage_margin;
extern float upper_thres_voltage_margin;
extern float outvolt_max;
extern float outvolt_min;
extern Uint16 faultDetected;
extern void ledCtrl(Uint16 state);
extern void dispParamsOnLED(void);
extern void SineTriangleToSVPWM(float, float, float, float *, float *, float *);
extern float ntcResistanceToTemperature(float,float,float,float);
extern float voltsInvToModulationIndexSVPWM(float, float);
extern void resetVariables();

// I2C initialization
extern void I2CA_Init(void);

// RTC
extern void RTC_WriteByte(Uint16, Uint16); // First argument is address, second argument is data
extern Uint16 RTC_ReadByte(Uint16);        // Argument is the address from where byte has to be read, Functions returns the read byte

// EEPROM
extern void EEPROM_WriteByte(Uint16, Uint16); // First argument is address, second argument is data
extern Uint16 EEPROM_ReadByte(Uint16);        // Argument is the address from where byte has to be read, Functions returns the read byte

// conversions
extern Uint16 Binary2BCD(Uint16 );
extern Uint16 BCD2Binary(Uint16 );

// get fault from EEPROM
extern void getPrevFaultFromEEPROM(Uint16);
//---------------------------------------------------------------------------
// CLA Function Prototypes
//
extern interrupt void Cla1Task1();
extern interrupt void Cla1Task2();
extern interrupt void Cla1Task3();
extern interrupt void Cla1Task4();
extern interrupt void Cla1Task5();
extern interrupt void Cla1Task6();
extern interrupt void Cla1Task7();
extern interrupt void Cla1Task8();


//---------------------------------------------------------------------------
// Global symbols defined in the linker command file
//
extern Uint16 cla1Funcs_loadstart;
extern Uint16 cla1Funcs_loadsize;
extern Uint16 cla1Funcs_runstart;
extern Uint16 secureRamFuncs_loadstart;
extern Uint16 secureRamFuncs_loadsize;
extern Uint16 secureRamFuncs_runstart;
extern Uint16 Cla1Prog_Start;


//---------------------------------------------------------------------------
// Global Variables References
//
extern const struct PIE_VECT_TABLE PieVectTableInit;    // PieVectTableInit is always extern

//---------------------------------------------------------------------------
// Macros
//

// The following pointer to a function call calibrates the ADC reference, 
// DAC offset, and internal oscillators
//#define Device_cal (void   (*)(void))0x070282

// The following pointers to functions calibrate the ADC linearity.  Use this
// in the AdcSetMode(...) function only
#define CalAdcaINL (void   (*)(void))0x0703B4
#define CalAdcbINL (void   (*)(void))0x0703B2
#define CalAdccINL (void   (*)(void))0x0703B0
#define CalAdcdINL (void   (*)(void))0x0703AE

//State variables
#define ActiveTripCond ((actualvalues.currsens > refValsActState.currRefMax) || (actualvalues.outputvolt > refValsActState.buckOutMax) || (actualvalues.dcLink > refValsActState.altOutMax) || (actualvalues.dcLink < refValsActState.altOutMin))
#define RestartFromTripCond ((actualvalues.currsens < refValsActState.currRefMax) && (actualvalues.outputvolt < refValsActState.buckOutMax) && (actualvalues.dcLink < refValsStartState.altOutMax) && (actualvalues.dcLink > refValsStartState.altOutMin))
#define StartupCond ((actualvalues.dcLink < refValsStartState.altOutMax) && (actualvalues.dcLink > refValsStartState.altOutMin) && (actualvalues.outputvolt < refValsStartState.buckOutMax))
#define output_volt_in_lim ((actualvalues.outputvolt >= outvolt_min) && (actualvalues.outputvolt <= outvolt_max))
#define overcurrentfault (actualvalues.currsens > refValsActState.currRefMax)
#define undervoltgefault (actualvalues.dcLink < refValsActState.altOutMin)
#define overvoltagefault (actualvalues.dcLink > refValsActState.altOutMax)
#define overoutvoltagefault (actualvalues.outputvolt > refValsActState.buckOutMax)
#define enablehardwaretrip (EPwm1Regs.TZEINT.bit.OST = 1)
#define disablehardwaretrip (EPwm1Regs.TZEINT.bit.OST = 0)
#define EPwmDown (EPwm1Regs.TZFLG.bit.OST == 1)
#define Tbprd 300

//Led turn off macros
#define gridPresent_led_turnon (GpioDataRegs.GPASET.bit.GPIO8 = 1)
#define gridPresent_led_turnoff (GpioDataRegs.GPACLEAR.bit.GPIO8 = 1)

#define pvPresent_led_turnon (GpioDataRegs.GPASET.bit.GPIO10 = 1)
#define pvPresent_led_turnoff (GpioDataRegs.GPACLEAR.bit.GPIO10 = 1)

#define invertorOn_led_turnon (GpioDataRegs.GPASET.bit.GPIO9 = 1)
#define invertorOn_led_turnoff (GpioDataRegs.GPACLEAR.bit.GPIO9 = 1)

#define fault_led_turnon (GpioDataRegs.GPASET.bit.GPIO11 = 1)
#define fault_led_turnoff (GpioDataRegs.GPACLEAR.bit.GPIO11 = 1)

// The following pointer to a function call looks up the ADC offset trim for a
// given condition. Use this in the AdcSetMode(...) function only.
#define GetAdcOffsetTrimOTP (Uint16 (*)(Uint16 OTPoffset))0x0703AC


//---------------------------------------------------------------------------
#endif  // end of LAB_H definition


//--- end of file -----------------------------------------------------
