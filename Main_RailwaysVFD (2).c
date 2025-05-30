
/**********************************************************************
Code   : Main C file for Variable Frequency Drive (VFD) for Indian Railways
Author : Veer Karan Goyal
**********************************************************************/
#include <RailwaysVFD_F280033.h>         // Main include file
#include "math.h"

#define yearMax 50
#define yearMin 24
#define prodYearT 2
#define prodYearO 1
//**********************************************************************
//--- Global Variables---
//**********************************************************************
unsigned char _LCD_CLEAR = 0x01;                // LCD Clear Command value

// Sampling Time for the Controller
float Ts = 0.0002 ;         // 200 us

Uint32 timeout=0;


float Vdc_sense = 0.0;
float Vdc_meas = 0.0;
float VdcCalFactor = 1.0;

float VFlt_sense = 0.0;
float VFlt_meas  = 0.0;
float VFltCalFactor = 1.0;

float frequencyCommand_sense = 0.0;
float frequencyCommand_meas = 0.0;

float deviceTempVolts_sense = 0.0;
float deviceTempVolts_meas = 0.0;

float vry_sense = 0.0;
float vry_meas  = 0.0;
float vryCalFactor = 1.0;
float vryOffset = 0.0;

float vrb_sense = 0.0;
float vrb_meas  = 0.0;
float vrbCalFactor = 1.0;
float vrbOffset = 0.0;

float ir_sense = 0.0;
float ir_meas  = 0.0;
float irCalFactor = 1.0;
float irOffset = 0.0;

float iy_sense = 0.0;
float iy_meas  = 0.0;
float iyCalFactor = 1.0;
float iyOffset = 0.0;

float ib_sense = 0.0;
float ib_meas  = 0.0;
float ibCalFactor = 1.0;
float ibOffset = 0.0;

Uint16 offsetSettingDone = 0;
Uint16 offsetSettingCount = 0;
Uint16 inputVoltageDetect = 0;


// Waveform viewing
float waveform1[100];
float waveform2[100];
Uint16 waveformCounter = 0;
Uint16 buffNum = 0;
// Variables for averaging/rms computing for input grid voltages, dc link voltage
// fanFailureVoltage, deviceTemperature
Uint16 buffNumData = 0;
float vrySum = 0.0;
float vryRms = 0.0;
float vrbSum = 0.0;
float vrbRms = 0.0;
float vdcSum = 0.0;
float vdcAvg = 0.0;
float VFltSum = 0.0;
float VFltAvg = 0.0;
float deviceTempVoltsSum = 0.0;
float deviceTempVoltsAvg = 0.0;

float freqInv = 0.0;
float Tinv    = 0.0;
float CycleCount = 0.0;

float buffNumDataVF = 0.0;
float irSum = 0.0;
float irRms = 0.0;
float iySum = 0.0;
float iyRms = 0.0;
float ibSum = 0.0;
float ibRms = 0.0;

float softChargeVoltage;
Uint16 faultPresent = 0;
Uint16 softwareTrip = 0;
Uint16 state = 0;
Uint16 faultCode=0;
Uint32 initialStateTimer=0;
Uint32 startUpStateTimer=0;
Uint32 tripStateTimer=0;
Uint32 controllerActiveStateTransitionTimer=0;
Uint32 faultTimer=0;
float voltsInv = 0;
Uint16 brakeChopperActive=0;
float time_step=0.1;
float deltaFreq = 0.0;
float Rntc;
float Temperature;
char Vry_arr[9],Vrb_arr[9],Vdc_arr[9],fanVolts_arr[9],Temp_arr[9],freqReq_arr[9];
char Ir_arr[9],Iy_arr[9],Ib_arr[9],Power_arr[],freqInv_arr[9],voltsInv_arr[9];
char stringPassword[3] = "00"; // Default password
int passwordValue = 0; // Integer value of the password
Uint16 yrCounter, monCounter, dtCounter;
Uint16 hrCounter, minCounter, secCounter;
char Time[9];
char Date[9];
Uint32 Counter=0;

////////////////////////////////////////////////////////////////////////for setting values in different states
Uint16 A1, A2, B1, B2, C1, C2, D1, D2, E1, E2, F1, F2, G1, G2, H1, H2, I1, I2, J1, J2, K1, K2, L1, L2, M1, M2;          // to store lowbyte and highbyte

Uint16 number1,number2,number3= 0;
char number1Str[] = " ";
char number2Str[] = " ";
char number3Str[] = " ";

Uint32 defaultVoltageValue = 380;
Uint32 defaultFrequencyValue = 50;
Uint32 defaultGridUVTripVoltage = 295;
Uint32 defaultGridLowerStartVoltage = 300;
Uint32 defaultGridUpperStartVoltage = 475;
Uint32 defaultGridOVTripVoltage = 480;
Uint32 defaultDCLinkOVTripVoltage = 660;
Uint32 defaultDCLinkUVTripVoltage = 300;
Uint32 defaultDCLinkUpperStartVoltage = 650;
Uint32 defaultLoadCurrentSoftwareTripLimit = 35;
Uint32 defaultLoadCurrentHardwareTripLimit = 45;
Uint32 defaultPowerDeviceOverTemperatureTripLimit = 75;  // 75 degree celcius
Uint32 defaultPowerDeviceStartTemperatureLimit = 70;  // 70 degree celciu

Uint32 eepromRatedFrequency=0;
Uint32 eepromRatedVoltage=0;
Uint32 eepromGridUVTripVoltage=0;
Uint32 eepromGridLowerStartVoltage=0;
Uint32 eepromGridUpperStartVoltage=0;
Uint32 eepromGridOVTripVoltage=0;
Uint32 eepromDCLinkOVTripVoltage=0;
Uint32 eepromDCLinkUVTripVoltage=0;
Uint32 eepromDCLinkUpperStartVoltage=0;
Uint32 eepromLoadCurrentSoftwareTripLimit=0;
Uint32 eepromLoadCurrentHardwareTripLimit=0;
Uint32 eepromPowerDeviceOverTemperatureTripLimit=0;
Uint32 eepromPowerDeviceStartTemperatureLimit=0;

char eepromRatedFrequencyStr[] = "   ";
char eepromRatedVoltageStr[] = "   ";
char eepromGridUVTripVoltageStr[] = "   ";
char eepromGridLowerStartVoltageStr[] = "   ";
char eepromGridOVTripVoltageStr[] = "   ";
char eepromGridUpperStartVoltageStr[] = "   ";
char eepromDCLinkOVTripVoltageStr[] = "   ";
char eepromDCLinkUVTripVoltageStr[] = "   ";
char eepromDCLinkUpperStartVoltageStr[] = "   ";
char eepromLoadCurrentSoftwareTripLimitStr[] = "   ";
char eepromLoadCurrentHardwareTripLimitStr[] = "   ";
char eepromPowerDeviceOverTemperatureTripLimitStr[] = "   ";
char eepromPowerDeviceStartTemperatureLimitStr[] = "   ";


Uint32 RatedVoltage = 0;
Uint32 RatedFrequency = 0;
Uint32 gridUVTripVoltageInt = 0;
Uint32 gridLowerStartVoltageInt = 0;
Uint32 gridUpperStartVoltageInt = 0;
Uint32 gridOVTripVoltageInt = 0;
Uint32 dcLinkOVTripVoltageInt = 0;
Uint32 dcLinkUVTripVoltageInt = 0;
Uint32 dcLinkUpperStartVoltageInt = 0;
Uint32 loadCurrentSoftwareTripLimitInt = 0;
Uint32 loadCurrentHardwareTripLimitInt = 0;
Uint32 powerDeviceOverTemperatureTripLimitInt = 0;
Uint32 powerDeviceStartTemperatureLimitInt = 0;

char RatedVoltageStr[] = "   ";
char RatedFrequencyStr[] = "   ";
char gridUVTripVoltageIntStr[] = "   ";
char gridLowerStartVoltageIntStr[] = "   ";
char gridUpperStartVoltageIntStr[] = "   ";
char gridOVTripVoltageIntStr[] = "   ";
char dcLinkOVTripVoltageIntStr[] = "   ";
char dcLinkUVTripVoltageIntStr[] = "   ";
char dcLinkUpperStartVoltageIntStr[] = "   ";
char loadCurrentSoftwareTripLimitIntStr[] = "   ";
char loadCurrentHardwareTripLimitIntStr[] = "   ";
char powerDeviceOverTemperatureTripLimitIntStr[] = "   ";
char powerDeviceStartTemperatureLimitIntStr[] = "   ";


Uint16 backPressDetected = 0;
Uint16 enteredFirstTime = 0;
Uint16 backTimer =0;

float ma = 0;
float mb = 0;
float mc = 0;
float ma_a_SVPWM = 0;
float ma_b_SVPWM = 0;
float ma_c_SVPWM = 0;
Uint16 a = 0;
Uint16 b = 0;
Uint16 c = 0;
float ma_a = 0;
float ma_b = 0;
float ma_c = 0;

// Sine Generation
float frequency = 0;
//float pi = 3.1416;
float modulationIndex = 1.0;
Uint16 cycleCounter = 0;
float theta = 0;

// DAC variables
float dacVoltage = 0;
Uint16 dacNumber = 0;

float irOCHVol    = 0;
float irOCLVol    = 0;
Uint16  irOCH       = 0;
Uint16  irOCL       = 0;

float iyOCHVol    = 0;
float iyOCLVol    = 0;
Uint16  iyOCH       = 0;
Uint16  iyOCL       = 0;

float ibOCHVol    = 0;
float ibOCLVol    = 0;
Uint16  ibOCH       = 0;
Uint16  ibOCL       = 0;

// TEST Variables
Uint16 goToFaultState = 0;
Uint16 goToTestState = 0;

// Sensor related variables
float currentSensingMultiplier = 85.7;
float hardwareTripVoltage = 0.0;

Uint16 page=0;
Uint16 downCounter = 0, upCounter = 0, backCounter = 0, enterCounter = 0;  //variables for push buttons
float Rntc;
float Bvalue = 3420.0;
float R25 = 5000.0;
float Room_Temperature = 298.15;
float powerDeviceTemperature=0.0;
float fanVoltsMax = 1.2;
float fanVoltsMin = 0.6;

Uint16 uiState = 0;
// Variable for motor power computation
float motorPowerSum = 0;
float motorPowerAvg = 0;
float instantaneousMotorPower = 0;
float vrn = 0;
float vyn = 0;
float vbn = 0;

// Fault Timings
Uint32 faultControllerActiveStateTimer=0;
// Current Fault Code
Uint16 currentFaultCode = 0;
char faultRead[78];                                     // variable to read fault-chunk from memory
char faultNameRead[]  ="                ";               // variable to read fault name from fault-chunk to display
char faultTimeStamp[] ="                 ";

// Variables for scaling up
float gridUVTripVoltage = 140.0;      // 295V full scale -> can be scaled linearly
float gridLowerStartVoltage = 150.0;  // 300V full scale -> can be scaled linearly
float gridUpperStartVoltage = 240.0;  // 480V full scale -> can be scaled linearly
float gridOVTripVoltage = 250.0;      // 485V full scale -> can be scaled linearly
float dcLinkOVTripVoltage = 350.0;    // 700V full scale -> can be scaled linearly
float dcLinkUVTripVoltage = 150.0;    // 380V full scale -> can be scaled linearly
float dcLinkUpperStartVoltage = 325.0; // 650V full scale -> can be scaled linearly
float loadCurrentSoftwareTripLimit = 20.0;  // 25A full scale
float loadCurrentHardwareTripLimit = 30.0;  // 35A full scale
float powerDeviceOverTemperatureTripLimit=75.0; // 80 degC full scale
float powerDeviceStartTemperatureLimit = 70.0;  // 75 degC full scale
float ratedMotorVoltage = 190.0;                // 380V full scale -> can be scaled linearly
float ratedMotorFrequency = 100.0;              // 100Hz full scale



// variables to set date and time

Uint16 yearCounter = yearMin, monthCounter = 1, dateCounter = 1;
Uint16 yrCounter, monCounter, dtCounter;
char yearCounterStr[] = "  ", monthCounterStr[] = "  ", dateCounterStr[] = "  ";
Uint16 hourCounter = 0, minuteCounter = 0, secondsCounter = 0;
Uint16 hrCounter, minCounter, secCounter;
char hourCounterStr[] = "  ", minuteCounterStr[] = "  ", secondsCounterStr[] = "  ";

/// variables to display date and time
Uint16 dateBCD, monthBCD, yearBCD, secondsBCD, minutesBCD, hoursBCD;
Uint16 date, month, year, seconds, minutes, hours;
char secondsString[] = "  ";
char minutesString[] = "  ";
char hoursString[]   = "  ";
char dateString[]    = "  ";
char monthString[]   = "  ";
char yearString[]    = "  ";


// Pin Out

// PWM Pins
/*
 GPIO0- PWM 1A
 GPIO1- PWM 1B
 GPIO2- PWM 2A
 GPIO3- PWM 2B
 GPIO4- PWM 3A
 GPIO5- PWM 3B
 GPIO6- PWM 4A
 */
// ADC Pins
/*
 ADC A6 - input line to line voltage (Vry)
 ADC B2/C6 - input lint to line voltage (Vyb)
 ADC A2/B6/C9 - DC Link Voltage
 ADC A5/B12/C2 - Output AC Current (ir)
 ADC A1/B7 - Output AC Current (iy)
 ADC A12/C1 - Output AC Current (ib)
 ADC A7/C3 - Analog input for VFD frequency and voltage setting
 ADC A11/B11/C0 - Power Semiconductor Module Temperature sensing
 ADC A14/B14/C4 - Voltage for fan failure detection
*/

// DAC Pins
/*
 DACA_OUT - Analog output voltage containing frequency and voltage information
 */

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

// I2C Communication
/*
 GPIO18 - SCL
 GPIO19 - SDA
 */

//#########################################################################
/********************************************************************
* Function: main()
**********************************************************************/
void main(void)
{

////--- CPU Initialization
    InitSysCtrl();                      // Initialize the CPU (FILE: f28002x_sysCtrl.c)

    InitGpio();                         // Initialize the shared GPIO pins (FILE: f28002x_gpio.c)

    DINT;

    InitPieCtrl();                      // Initialize and enable the PIE (FILE: PieCtrl.c)

// Initialize the Flash

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    //
    // Call Flash Initialization to setup flash wait states. This function must
    // reside in RAM.
    //
    InitFlash();


// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
//
    InitPieVectTable();

//
// Map ISR functions
//
    EALLOW;
    PieVectTable.ADCA1_INT = &ADCA1_ISR;     // Function for ADCA interrupt 1
    EDIS;

    //I2CA_Init();
    LcdInit();

    Delay_ms(100);


//--- Peripheral Initialization
       initADC();                                      // Initialize the ADC-A (FILE: Adc.c)
       //codeFlowTest = 6;
       InitEPwm();                                     // Initialize the EPwm (FILE: EPwm.c)
       //codeFlowTest = 7;
       InitDaca();

       I2CA_Init();

       IER |= 0x0001;                                  // Enable INT1 in IER to enable PIE group 1
       //
       // Enable PIE interrupt
       //
       PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

       PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Must acknowledge the PIE group

//--- Enable global interrupts
       EINT;           // Enable Global interrupt INTM
       ERTM;           // Enable Global realtime interrupt DBGM

       Lcd_Cmd(_LCD_CLEAR);
       Lcd_out(1, 1, "EWE: VFD");
       Delay_ms(1000);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////// Read configuration from memory

        if(EEPROM_ReadByte(ratedVoltageLocation) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+1) == 0xFF)
        {
            eepromRatedVoltage = defaultVoltageValue;
        }
        else
        {
            A2 = EEPROM_ReadByte(ratedVoltageLocation);             //reads voltage
            A1 = EEPROM_ReadByte(ratedVoltageLocation+1);
            eepromRatedVoltage = ((Uint16) A1) | ((Uint16) (A2 << 8));
        }

        if(EEPROM_ReadByte(ratedVoltageLocation+2) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+3) == 0xFF)
        {
            eepromRatedFrequency = defaultFrequencyValue;
        }
        else
        {
            B2 = EEPROM_ReadByte(ratedVoltageLocation+2);           //reads frequency
            B1 = EEPROM_ReadByte(ratedVoltageLocation+3);
            eepromRatedFrequency = ((Uint16) B1) | ((Uint16) (B2 << 8));
        }

        if(EEPROM_ReadByte(ratedVoltageLocation+4) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+5) == 0xFF)
        {
            eepromGridUVTripVoltage= defaultGridUVTripVoltage;
        }
        else
        {
            C2 = EEPROM_ReadByte(ratedVoltageLocation+4);           //reads gridUVTripVoltage
            C1 = EEPROM_ReadByte(ratedVoltageLocation+5);
            eepromGridUVTripVoltage = ((Uint16) C1) | ((Uint16) (C2 << 8));
        }
        if(EEPROM_ReadByte(ratedVoltageLocation+6) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+7) == 0xFF)
        {
            eepromGridLowerStartVoltage = defaultGridLowerStartVoltage;
        }
        else
        {
            D2 = EEPROM_ReadByte(ratedVoltageLocation+6);           //reads GridLowerStartVoltage
            D1 = EEPROM_ReadByte(ratedVoltageLocation+7);
            eepromGridLowerStartVoltage = ((Uint16) D1) | ((Uint16) (D2 << 8));
        }
        if(EEPROM_ReadByte(ratedVoltageLocation+8) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+9) == 0xFF)
        {
            eepromGridUpperStartVoltage= defaultGridUpperStartVoltage;
        }
        else
        {
            E2 = EEPROM_ReadByte(ratedVoltageLocation+8);           //reads GridUpperStartVoltage
            E1 = EEPROM_ReadByte(ratedVoltageLocation+9);
            eepromGridUpperStartVoltage = ((Uint16) E1) | ((Uint16) (E2 << 8));
        }
        if(EEPROM_ReadByte(ratedVoltageLocation+10) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+11) == 0xFF)
        {
            eepromGridOVTripVoltage= defaultGridOVTripVoltage;
        }
        else
        {
            F2 = EEPROM_ReadByte(ratedVoltageLocation+10);           //reads GridOVTripVoltage
            F1 = EEPROM_ReadByte(ratedVoltageLocation+11);
            eepromGridOVTripVoltage = ((Uint16) F1) | ((Uint16) (F2 << 8));
        }
        if(EEPROM_ReadByte(ratedVoltageLocation+12) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+13) == 0xFF)
        {
            eepromDCLinkOVTripVoltage= defaultDCLinkOVTripVoltage;
        }
        else
        {
            G2 = EEPROM_ReadByte(ratedVoltageLocation+12);           //reads DCLinkOVTripVoltage
            G1 = EEPROM_ReadByte(ratedVoltageLocation+13);
            eepromDCLinkOVTripVoltage = ((Uint16) G1) | ((Uint16) (G2 << 8));
        }
        if(EEPROM_ReadByte(ratedVoltageLocation+14) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+15) == 0xFF)
        {
            eepromDCLinkUVTripVoltage= defaultDCLinkUVTripVoltage;
        }
        else
        {
            H2 = EEPROM_ReadByte(ratedVoltageLocation+14);           //reads DCLinkUVTripVoltage
            H1 = EEPROM_ReadByte(ratedVoltageLocation+15);
            eepromDCLinkUVTripVoltage = ((Uint16) H1) | ((Uint16) (H2 << 8));
        }
        if(EEPROM_ReadByte(ratedVoltageLocation+16) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+17) == 0xFF)
        {
            eepromDCLinkUpperStartVoltage= defaultDCLinkUpperStartVoltage;
        }
        else
        {
            I2 = EEPROM_ReadByte(ratedVoltageLocation+16);           //reads DCLinkUpperStartVoltage
            I1 = EEPROM_ReadByte(ratedVoltageLocation+17);
            eepromDCLinkUpperStartVoltage = ((Uint16) I1) | ((Uint16) (I2 << 8));
        }

        if(EEPROM_ReadByte(ratedVoltageLocation+18) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+19) == 0xFF)
        {
            eepromLoadCurrentSoftwareTripLimit= defaultLoadCurrentSoftwareTripLimit;
        }
        else
        {
            J2 = EEPROM_ReadByte(ratedVoltageLocation+18);           //reads LoadCurrentSoftwareTripLimit
            J1 = EEPROM_ReadByte(ratedVoltageLocation+19);
            eepromLoadCurrentSoftwareTripLimit = ((Uint16) J1) | ((Uint16) (J2 << 8));
        }

        if(EEPROM_ReadByte(ratedVoltageLocation+20) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+21) == 0xFF)
        {
            eepromLoadCurrentHardwareTripLimit= defaultLoadCurrentHardwareTripLimit;
        }
        else
        {
            K2 = EEPROM_ReadByte(ratedVoltageLocation+20);           //reads gridUVTripVoltage
            K1 = EEPROM_ReadByte(ratedVoltageLocation+21);
            eepromLoadCurrentHardwareTripLimit = ((Uint16) K1) | ((Uint16) (K2 << 8));
        }

        if(EEPROM_ReadByte(ratedVoltageLocation+22) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+23) == 0xFF)
        {
            eepromPowerDeviceOverTemperatureTripLimit= defaultPowerDeviceOverTemperatureTripLimit;
        }
        else
        {
            L2 = EEPROM_ReadByte(ratedVoltageLocation+22);           //reads gridUVTripVoltage
            L1 = EEPROM_ReadByte(ratedVoltageLocation+23);
            eepromPowerDeviceOverTemperatureTripLimit = ((Uint16) L1) | ((Uint16) (L2 << 8));
        }

        if(EEPROM_ReadByte(ratedVoltageLocation+24) == 0xFF || EEPROM_ReadByte(ratedVoltageLocation+25) == 0xFF)
        {
            eepromPowerDeviceStartTemperatureLimit= defaultPowerDeviceStartTemperatureLimit;
        }
        else
        {
            M2 = EEPROM_ReadByte(ratedVoltageLocation+24);           //reads gridUVTripVoltage
            M1 = EEPROM_ReadByte(ratedVoltageLocation+25);
            eepromPowerDeviceStartTemperatureLimit = ((Uint16) M1) | ((Uint16) (M2 << 8));
        }

       ratedMotorVoltage = eepromRatedVoltage;
       ratedMotorFrequency = eepromRatedFrequency;
       gridUVTripVoltage = eepromGridUVTripVoltage;
       gridLowerStartVoltage = eepromGridLowerStartVoltage;
       gridUpperStartVoltage = eepromGridOVTripVoltage;
       gridOVTripVoltage = eepromGridOVTripVoltage;
       dcLinkOVTripVoltage = eepromDCLinkOVTripVoltage;
       dcLinkUVTripVoltage = eepromDCLinkUVTripVoltage;
       dcLinkUpperStartVoltage = eepromDCLinkUpperStartVoltage;
       loadCurrentSoftwareTripLimit = eepromLoadCurrentSoftwareTripLimit;
       loadCurrentHardwareTripLimit = eepromLoadCurrentHardwareTripLimit;
       powerDeviceOverTemperatureTripLimit = eepromPowerDeviceOverTemperatureTripLimit;
       powerDeviceStartTemperatureLimit = eepromPowerDeviceStartTemperatureLimit;


       state = initialState;
       uiState = defaultState;
       page=1;







       while(1)
           {

               Rntc = (25500.0 / deviceTempVoltsAvg) - 10200.0;

               powerDeviceTemperature = ntcResistanceToTemperature(Rntc, R25, Bvalue, Room_Temperature);

               ftoa(vryRms, Vry_arr,1);
               ftoa(vrbRms, Vrb_arr,1);
               ftoa(vdcAvg, Vdc_arr,1);
               ftoa(VFltAvg, fanVolts_arr,2);
               ftoa(powerDeviceTemperature, Temp_arr,1);
               ftoa(frequencyCommand_meas, freqReq_arr,1);
               ftoa(irRms, Ir_arr,1);
               ftoa(iyRms, Iy_arr,1);
               ftoa(ibRms, Ib_arr,1);
               ftoa(motorPowerAvg, Power_arr,3);
               ftoa(freqInv, freqInv_arr,1);
               ftoa(voltsInv, voltsInv_arr,1);

             switch (uiState)
             {
               case defaultState:
               {
//                   downCounter=0;
//                   upCounter=0;
                   while(down == 0)                      //  down button is pressed
                   {
                       Delay_ms(1);
                       downCounter =downCounter + 1;
                       if(downCounter==500)        //checks if button is pressed for 1 sec
                       {
                           page = (page - 1 ==1 ) ? page - 1 : NUM_PAGES;
                           downCounter = 0;
                           break;
                       }
                   }
                   while(up == 0)                      //  up button is pressed
                   {
                       Delay_ms(1);
                       upCounter = upCounter + 1;
                       if(upCounter==500)         //checks if button is pressed for 1 sec
                       {
                           page = (page + 1 <= NUM_PAGES) ? page + 1 : 1;
                           upCounter = 0;
                           break;
                       }
                   }

                   while(back == 0)                   // change uiState to restrictedAccess.
                   {
                       Delay_ms(1);
                       backCounter = backCounter + 1;
                       if(backCounter==500)
                       {
                           uiState = Measurements;
                           backCounter = 0;
                           break;
                       }
                   }

                  switch(page)
                   {
                       case 1:
                               Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                               Lcd_out(1, 1, "Vry,    Vrb ");
                               Lcd_out(2, 1, Vry_arr);
                               Lcd_out(2, 6,"V, ");
                               Lcd_out(2, 9, Vrb_arr);
                               Lcd_out(2, 15,"V");
                               Delay_ms(20);    //there is 20ms delay after every page
                               break;

                       case 2:
                               Lcd_Cmd (_LCD_CLEAR);                     //clear display screen
                               Lcd_out(1, 1, "Vdc   ,fanVolts ");
                               Lcd_out(2, 1, Vdc_arr);
                               Lcd_out(2, 6,"V, ");
                               Lcd_out(2, 8, fanVolts_arr);
                               Lcd_out(2, 14,"V");
                               Delay_ms(20);    //there is 20ms delay after every page
                               break;

                       case 3:
                               Lcd_Cmd (_LCD_CLEAR);                     //clear display screen
                               Lcd_out(1, 1, "Temp  , freqReq");
                               Lcd_out(2, 1, Temp_arr);
                               Lcd_out(2, 6,"C, ");
                               Lcd_out(2, 9, freqReq_arr);
                               Lcd_out(2, 14,"Hz");
                               Delay_ms(20);    //there is 20ms delay after every page
                               break;

                       case 4:
                               Lcd_Cmd (_LCD_CLEAR);                     //clear display screen
                               Lcd_out(1, 1, "Ir    ,Iy  ");
                               Lcd_out(2, 1, Ir_arr);
                               Lcd_out(2, 6,"A, ");
                               Lcd_out(2, 8, Iy_arr);
                               Lcd_out(2, 14,"A");
                               Delay_ms(20);    //there is 20ms delay after every page
                               break;

                       case 5:
                               Lcd_Cmd (_LCD_CLEAR);                     //clear display screen
                               Lcd_out(1, 1, "Ib    ,Power  ");
                               Lcd_out(2, 1, Ib_arr);
                               Lcd_out(2, 6,"A, ");
                               Lcd_out(2, 8, Power_arr);
                               Lcd_out(2, 14,"kW");
                               Delay_ms(20);    //there is 20ms delay after every page
                               break;

                       case 6:
                               Lcd_Cmd (_LCD_CLEAR);                     //clear display screen
                               Lcd_out(1, 1, "freqInv,voltsInv");
                               Lcd_out(2, 1, freqInv_arr);
                               Lcd_out(2, 6,"Hz, ");
                               Lcd_out(2, 10, voltsInv_arr);
                               Lcd_out(2, 16,"V");
                               Delay_ms(20);    //there is 20ms delay after every page
                               break;

                       case 7:
                              readDateTimeRTC();
                              Lcd_Cmd (_LCD_CLEAR);
                              Lcd_out(1, 1, "Date");
                              Lcd_out(1, 6, Date);
                              Lcd_out(2, 1, "Time");
                              Lcd_out(2, 6, Time);
                              Delay_ms(20);    //there is 20ms delay after every page
                              break;


                       default:
                               break;

                   }


                   if(faultCode!=0)
                   {
                       uiState = faultState;
                   }


                   break;
               }

                  case faultState:
                  {         //Depending on the faultCode, display the fault information to the user
                       if(faultCode == 1 )
                       {
                           Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "Grid Over");
                           Lcd_out(2, 1, "Voltage");

                       }
                       else if(faultCode == 2 )
                       {
                         Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                        Lcd_out(1, 1, "Grid Under");
                        Lcd_out(2, 1, "Voltage");

                       }
                       else if(faultCode == 3 )
                       {
                            Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "DC Link Over");
                           Lcd_out(2, 1, "Voltage");

                       }
                       else if(faultCode == 4 )
                       {
                           Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "DC Link Under");
                           Lcd_out(2, 1, "Voltage");

                       }
                       else if(faultCode == 5 )
                       {
                            Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "Load Over");
                           Lcd_out(2, 1, "Current");

                       }
                       else if(faultCode == 6 )
                       {
                           Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "Device Over");
                           Lcd_out(2, 1, "powerDeviceTemperature");

                       }
                       else if(faultCode == 7 )
                       {
                           Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "Fan Failure");
                           Lcd_out(2, 1, "Detection");

                       }
                       else if(faultCode == 8 )
                       {
                           Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "softCharge");
                           Lcd_out(2, 1, "failure");

                       }
                       else if(faultCode == 10)
                       {
                           Lcd_Cmd (_LCD_CLEAR);                      //clear display screen
                           Lcd_out(1, 1, "Hardware");
                           Lcd_out(2, 1, "Trip");

                       }
                       else if(faultCode == 0 )
                       {
                           uiState = defaultState;
                       }
                       break;
                  }

                  case Measurements:
                  {
                        Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                        Lcd_out(1, 2, "Measurements?");
                        Delay_ms(100);
                        while(1)
                        {
                            if(back == 0)
                            {
                                Delay_ms(1);
                                backCounter = backCounter + 1;
                                if(backCounter==500)
                                {
                                   uiState = restrictedAccess;
                                   backCounter = 0;
                                  break;
                                }
                             }
                            else if(enter == 0)
                            {
                                Delay_ms(1);
                                enterCounter = enterCounter + 1;
                                if(enterCounter==500)
                                {
                                uiState = defaultState;       // change uiState to Default
                                enterCounter = 0;
                                break;
                                }
                            }
                            else
                            {
                                backCounter = 0;
                                enterCounter = 0;
                            }
                        }
                        break;
                  }

                  case restrictedAccess:
                  {

                      Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                      Lcd_out(1, 2, "  Restricted");
                      Lcd_out(2, 3, "  Access?");
                      strcpy(stringPassword, "00");
                      passwordValue = 0;
//                      enteredFirstTime == 0;
//                      backTimer == 1;

                      Delay_ms(100);
                      while(1)
                      {
                          if(back == 0)
                          {
                              Delay_ms(1);
                              backCounter = backCounter + 1;
                              if(backCounter==500)
                              {
                              uiState = Measurements;
                              backCounter = 0;
                              break;
                              }
                          }
                          else if(enter == 0)
                          {
                              Delay_ms(1);
                              enterCounter = enterCounter + 1;
                              if(enterCounter==500)
                              {
                              uiState = enterPassword;
                              enterCounter = 0;
                              break;
                              }
                          }
                          else
                          {
                               upCounter = 0;
                               downCounter = 0;
                               backCounter = 0;
                               enterCounter = 0;
                          }
                      }
                      break;
                  }

                  case enterPassword:
                  {
                      Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                      Lcd_out(1, 2, "Enter Password:");
                      Lcd_out(2, 7, stringPassword );
                      Delay_ms(100);

                      while(1)
                      {
                          if(up == 0)
                          {
                             upCounter++;
                             Delay_ms(1);
                             if(upCounter==500)
                             {
                                 passwordValue = (passwordValue + 1 <= 99) ? passwordValue + 1 : 00;
                                 passwordValue = passwordValue % 100;     // Increment password value, wrap around at 99
                                 intToStr(passwordValue, stringPassword, 2);     // Convert password value back to a string
                                 break;
                             }
                          }
                          else if(down == 0)
                          {
                              downCounter++;
                              Delay_ms(1);
                              if(downCounter==500)
                              {
                                  passwordValue = (passwordValue - 1 >=00) ? passwordValue - 1 : 99;
                                  passwordValue =  passwordValue % 100;    // Decrement password value
                                  intToStr(passwordValue, stringPassword, 2);   // Convert password value back to a string
                                  break;
                              }

                          }
                          else if(back == 0)
                          {
                              backCounter = backCounter + 1;
                              if(backCounter==500)
                              {
                               uiState = restrictedAccess;             //change uiState to restrictedAccess
                               backCounter = 0;
                               break;
                              }
                          }
                          else if(enter == 0)
                          {
                              Delay_ms(1);
                              enterCounter = enterCounter + 1;
                              if(enterCounter==500)
                              {
                              uiState = passwordCheck;                //change uiState to Default
                              enterCounter = 0;
                              break;
                              }
                          }
                          else
                            {
                                 upCounter = 0;
                                 downCounter = 0;
                                 backCounter = 0;
                                 enterCounter = 0;
                            }
                      }
                       break;
                  }    //enterPassword case ends

                 case passwordCheck:
                 {
                     if(passwordValue == correctPassword)
                     {
                         Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                         Lcd_out(1, 2, "Access Granted");
                         Delay_ms(100);
                         while(1)
                         {
                             Delay_ms(1);
                             Counter = Counter + 1;
                             if(Counter==2000)
                             {
                                 uiState = setDateTime;
                                 Counter = 0;
                                 break;
                             }
                         }
                     }
                     if(passwordValue != correctPassword)
                     {
                         Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                         Lcd_out(1, 2, "Access Denied");
                         Delay_ms(100);
                         while(1)
                         {
                             Delay_ms(1);
                             Counter = Counter + 1;
                             if(Counter==2000)
                             {
                                 uiState = restrictedAccess;
                                 Counter = 0;
                                 break;
                             }
                         }
                     }
                     break;
                 }

                 case setDateTime:
                 {
                     if(enteredFirstTime == 0 || backTimer == 1)
                     {
                        Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                        Lcd_out(1, 2, "Set Date Time?");
                        backTimer =0;
                        enteredFirstTime=1;
                        dateCounter = 1;
                        monthCounter = 1;
                        yearCounter = yearMin;
                        hourCounter = 0;
                        minuteCounter = 0;
                        secondsCounter = 0;
                        upCounter= 0;
                        downCounter= 0;
                        enterCounter = 0;
                        backCounter = 0;
                        Delay_ms(1000);
                     }

                     while(1)
                     {
                           if(up == 0)
                           {
                                upCounter= upCounter + 1;
                                Delay_ms(1);
                                if(upCounter== 500)
                                {
                                    uiState = setRatedVoltage;
                                    upCounter= 0;
                                    enteredFirstTime = 0;
                                    backTimer = 1;
                                    break;
                                }
                           }
                           else if(down == 0)
                           {
                               downCounter= downCounter + 1;
                               Delay_ms(1);
                               if(downCounter== 500)
                               {
                                   uiState = setRatedFrequency;
                                   downCounter= 0;
                                   enteredFirstTime = 0;
                                   backTimer = 1;
                                   break;
                               }
                           }
                           else if(back == 0)
                           {
                               Delay_ms(1);
                               backCounter = backCounter + 1;
                               if(backCounter==500)
                               {
                                   uiState = restrictedAccess;
                                   backCounter = 0;
                                   enteredFirstTime = 0;
                                   break;
                               }
                           }
                           else if(enter == 0)
                           {
                              Delay_ms(1);
                              enterCounter = enterCounter + 1;
                              if(enterCounter==500)
                              {
                                  Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                                  Lcd_out(1, 1, "Date-dd:mm:yy");
                                  Lcd_out(2, 1, "Time-dd:mm:yy");
                                  Delay_ms(1000);
                                  uiState = setDate;
                                  enterCounter = 0;
                                  enteredFirstTime = 0;
                                  break;
                              }
                           }
                           else
                           {
                               upCounter= 0;
                               downCounter= 0;
                               enterCounter = 0;
                               backCounter = 0;
                           }
                     }
                   break;
                 }

                 case setDate:
                 {
                     while(1)
                     {
                         if(enteredFirstTime == 0  )
                         {
                             Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                             Lcd_out(1, 1, "Set Date:");
                             intToStr(dateCounter, dateCounterStr, 2);
                             Lcd_out(2, 1, dateCounterStr);
                             enteredFirstTime = 1;
                             Delay_ms(500);

                         } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setDateTime;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end

                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end

                         if(up==0)
                         {
                             upCounter= upCounter+ 1;
                             Delay_ms(1);
                             if(upCounter== 500)
                             {
                                 dateCounter=dateCounter+1;
                                 if(dateCounter==32)
                                 {
                                     dateCounter=1;
                                 }
                                 upCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end

                         } // if end
                         else
                         {
                             upCounter= 0;
                         } // else end
                         if(down==0)
                         {
                             downCounter= downCounter+ 1;
                             Delay_ms(1);
                             if(downCounter== 500)
                             {
                                 dateCounter=dateCounter-1;
                                 if(dateCounter==0)
                                 {
                                     dateCounter=31;
                                 }
                                 downCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end

                         } // if end
                         else
                         {
                             downCounter= 0;
                         } // else end
                     } // date set while end

                     while(1)
                     {
                       if(enteredFirstTime == 0)
                       {
                            Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                            Lcd_out(1, 1, "Set Date:");
                            intToStr(dateCounter, dateCounterStr, 2);
                            Lcd_out(2, 1, dateCounterStr);
                            intToStr(monthCounter, monthCounterStr, 2);
                            Lcd_out(2, 3, "/");
                            Lcd_out(2, 4, monthCounterStr);
                            Delay_ms(500);
                            enteredFirstTime = 1;
                        } // if end
                        if(back==0)
                        {
                            backCounter= backCounter+ 1;
                            Delay_ms(1);
                            if(backCounter== 500)
                            {
                                uiState = setDateTime;
                                backCounter= 0;
                                backTimer = 1;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                            backCounter= 0;
                        } // else end
                        if(enter==0)
                        {
                            enterCounter= enterCounter+ 1;
                            Delay_ms(1);
                            if(enterCounter== 500)
                            {
                                enterCounter= 0;
                                enteredFirstTime = 0;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                                enterCounter= 0;
                        } // else end
                        if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter == 500)
                            {
                                monthCounter=monthCounter+1;
                                if(monthCounter==13)
                                {
                                   monthCounter=1;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter == 500)
                            {
                                monthCounter=monthCounter-1;
                                if(monthCounter==0)
                                {
                                   monthCounter=12;
                                }
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                     } // set month while end
                     while(1)                          //set Time
                     {
                         if(enteredFirstTime == 0)
                         {
                              Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                              Lcd_out(1, 1, "Set Date:");
                              intToStr(dateCounter, dateCounterStr, 2);
                              Lcd_out(2, 1, dateCounterStr);
                              intToStr(monthCounter, monthCounterStr, 2);
                              Lcd_out(2, 3, "/");
                              Lcd_out(2, 4, monthCounterStr);
                              intToStr(yearCounter, yearCounterStr, 2);
                              Lcd_out(2, 6, "/");
                              Lcd_out(2, 7, yearCounterStr);
                              Delay_ms(500);
                              enteredFirstTime = 1;
                          } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setDateTime;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 uiState =  dateTimeCheck;
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end

                         if(up==0)
                         {
                             upCounter= upCounter+ 1;
                             Delay_ms(1);
                             if(upCounter== 500)
                             {
                                 yearCounter=yearCounter+1;
                                 if(yearCounter == (yearMax+1))     //yearMax is 50
                                 {
                                    yearCounter=yearMin;             //yearMin is 21
                                 }
                                 upCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end
                         } // if end
                         else
                         {
                              upCounter= 0;
                         } // else end
                         if(down==0)
                         {
                             downCounter= downCounter+ 1;
                             Delay_ms(1);
                             if(downCounter== 500)
                             {
                                 yearCounter=yearCounter-1;
                                 if(yearCounter == (yearMin-1))
                                 {
                                    yearCounter=yearMax;
                                 }
                                 uiState= dateTimeCheck;
                                 downCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end
                         } // if end
                         else
                         {
                             downCounter= 0;
                         } // else end
                     } // year while end

                     break;
                 }

                 case setTime:
                 {
                     while(1)                       ///////////////set Time
                     {
                         if(enteredFirstTime == 0)
                         {
                             Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                             Lcd_out(1, 1, "Set Time:");
                             intToStr(hourCounter, hourCounterStr, 2);
                             Lcd_out(2, 1, hourCounterStr);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                         } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState= setDateTime;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end
                         if(up==0)
                         {
                             upCounter= upCounter+ 1;
                             Delay_ms(1);
                             if(upCounter== 500)
                             {
                                 hourCounter=hourCounter+1;
                                 if(hourCounter==24)
                                 {
                                    hourCounter=0;
                                 }
                                 upCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end
                         } // if end
                         else
                         {
                             upCounter= 0;
                         } // else end
                         if(down==0)
                         {
                             downCounter= downCounter+ 1;
                             Delay_ms(1);
                             if(downCounter== 500)
                             {
                                 if(hourCounter==0)
                                 {
                                    hourCounter=24;
                                 }
                                 hourCounter=hourCounter-1;
                                 downCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end
                         } // if end
                         else
                         {
                             downCounter= 0;
                         } // else end
                     } // set hours while end

                     while(1)
                     {
                        if(enteredFirstTime == 0)
                        {
                            Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                            Lcd_out(1, 1, "Set Time:");
                            intToStr(hourCounter, hourCounterStr, 2);
                            Lcd_out(2, 1, hourCounterStr);
                            intToStr(minuteCounter, minuteCounterStr, 2);
                            Lcd_out(2, 3, ":");
                            Lcd_out(2, 4, minuteCounterStr);
                            Delay_ms(500);
                            enteredFirstTime = 1;
                        } // if end

                        if(back==0)
                        {
                            backCounter= backCounter+ 1;
                            Delay_ms(1);
                            if(backCounter== 500)
                            {
                                uiState= setDateTime;
                                backCounter= 0;
                                backTimer = 1;
                                break;
                            } // if end

                        } // if end
                        else
                        {
                            backCounter= 0;
                        } // else end
                        if(enter==0)
                        {
                            enterCounter= enterCounter+ 1;
                            Delay_ms(1);
                            if(enterCounter== 500)
                            {
                                enterCounter = 0;
                                enteredFirstTime = 0;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                            enterCounter = 0;
                        } // else end

                        if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                minuteCounter=minuteCounter+1;
                                if(minuteCounter==60)
                                {
                                    minuteCounter=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end

                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(minuteCounter==0)
                                {
                                    minuteCounter=60;
                                }
                                minuteCounter=minuteCounter-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end

                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                     } // set minutes while end

                     while(1)
                     {
                        if(enteredFirstTime == 0)
                        {
                            Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                            Lcd_out(1, 1, "Set Time:");
                            intToStr(hourCounter, hourCounterStr, 2);
                            Lcd_out(2, 1, hourCounterStr);
                            intToStr(minuteCounter, minuteCounterStr, 2);
                            Lcd_out(2, 3, ":");
                            Lcd_out(2, 4, minuteCounterStr);
                            intToStr(secondsCounter, secondsCounterStr, 2);
                            Lcd_out(2, 6, ":");
                            Lcd_out(2, 7, secondsCounterStr);
                            Delay_ms(500);
                            enteredFirstTime = 1;
                        } // if end

                        if(back==0)
                        {
                            backCounter= backCounter+ 1;
                            Delay_ms(1);
                            if(backCounter== 500)
                            {
                                uiState= setDateTime;
                                backCounter= 0;
                                backTimer = 1;
                                break;

                            } // if end


                        } // if end
                        else
                        {
                            backCounter= 0;
                        } // else end
                        if(enter==0)
                        {
                            enterCounter= enterCounter+ 1;
                            Delay_ms(1);
                            if(enterCounter== 500)
                            {
                                uiState = doneSetting;        //
                                enterCounter= 0;
                                enteredFirstTime = 0;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                            enterCounter= 0;
                        } // else end

                        if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                secondsCounter=secondsCounter+1;
                                if(secondsCounter==60)
                                {
                                    secondsCounter=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end

                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(secondsCounter==0)
                                {
                                    secondsCounter=60;
                                }
                                secondsCounter=secondsCounter-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end

                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                     } // set seconds while end


                     break;

                 }

                 case doneSetting:
                 {
                     secCounter = Binary2BCD(secondsCounter);
                     minCounter = Binary2BCD(minuteCounter);
                     hrCounter  = Binary2BCD(hourCounter);
                     dtCounter  = Binary2BCD(dateCounter);
                     monCounter = Binary2BCD(monthCounter);
                     yrCounter  = Binary2BCD(yearCounter);

          //           //////PieCtrlRegs.PIEIER1.bit.INTx14 = 0;         // Disable IPC1 ISR
                     RTC_WriteByte(0x00,secCounter);
                     RTC_WriteByte(0x01,minCounter);
                     RTC_WriteByte(0x02,hrCounter);
                     RTC_WriteByte(0x04,dtCounter);
                     RTC_WriteByte(0x05,monCounter);
                     RTC_WriteByte(0x06,yrCounter);

                     //read date and time from RTC

                     secondsBCD = RTC_ReadByte(0x0000);
                     minutesBCD = RTC_ReadByte(0x0001);
                     hoursBCD   = RTC_ReadByte(0x0002);
                     dateBCD    = RTC_ReadByte(0x0004);
                     monthBCD   = RTC_ReadByte(0x0005);
                     yearBCD    = RTC_ReadByte(0x0006);
          //           //////PieCtrlRegs.PIEIER1.bit.INTx14 = 1;         // Enable IPC1 ISR

                     seconds    = BCD2Binary(secondsBCD);
                     minutes    = BCD2Binary(minutesBCD);
                     hours      = BCD2Binary(hoursBCD);
                     date       = BCD2Binary(dateBCD);
                     month      = BCD2Binary(monthBCD);
                     year       = BCD2Binary(yearBCD);

                     intToStr(seconds, secondsString, 2);
                     intToStr(minutes, minutesString, 2);
                     intToStr(hours  , hoursString  , 2);
                     intToStr(date   , dateString   , 2);
                     intToStr(month  , monthString  , 2);
                     intToStr(year   , yearString   , 2);

                     //display date and time on LCD
                     Lcd_Cmd(_LCD_CLEAR);
                     Lcd_out(1,1, "Time-");
                     Lcd_out(1,6, hoursString);
                     Lcd_out(1,8, ":");
                     Lcd_out(1,9, minutesString);
                     Lcd_out(1,11, ":");
                     Lcd_out(1,12, secondsString);
                     Lcd_out(2,1, "Date-");
                     Lcd_out(2,6, dateString);
                     Lcd_out(2,8, "/");
                     Lcd_out(2,9, monthString);
                     Lcd_out(2,11, "/");
                     Lcd_out(2,12, yearString);

                     Delay_ms(2000);

                     dateCounter = 1;
                     monthCounter = 1;
                     yearCounter = yearMin;
                     hourCounter = 0;
                     minuteCounter = 0;
                     secondsCounter = 0;
                     uiState= restrictedAccess;
                     break;
                 }

                 case dateTimeCheck:
                 {
                    if((!invalidDate))
                       {
                             Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                             Lcd_out(1, 1, "Date Set");

                             while(1)
                             {
                                 Delay_ms(1);
                                 Counter = Counter + 1;
                                 if(Counter==2000)
                                 {
                                     uiState =setTime;
                                     Counter = 0;
                                     break;
                                 }
                             }
                       }

                    else
                    {
                           Lcd_Cmd (_LCD_CLEAR);              //clear display screen
                           Lcd_out(1, 1, "Invalid Date");

                           while(1)
                           {
                               Delay_ms(1);
                               Counter = Counter + 1;
                               if(Counter==2000)
                               {
                                   uiState = setDateTime;
                                   Counter = 0;
                                   break;
                               }
                           }
                           dateCounter = 1;
                           monthCounter = 1;
                           yearCounter = yearMin;
                           hourCounter = 0;
                           minuteCounter = 0;
                           secondsCounter = 0;
                    }
                     break;
                 }

                 case setRatedVoltage:
                 {
                     Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                     Lcd_out(1, 4, "Set Rated");
                     Lcd_out(2, 4, "Voltage?");
                     Delay_ms(500);

                     while(1)
                     {
                         if(up == 0)
                         {
                             upCounter++;
                             if(upCounter==500)
                             {
                                 uiState = setRatedFrequency;
                                 upCounter = 0;
                                 break;
                             }

                         }
                         else if(down == 0)
                         {
                             downCounter++;
                             if(downCounter==500)
                              {
                                //  uiState = setgridUVTripVoltage;
                                  uiState = setdcLinkOVTripVoltage;
                                  downCounter = 0;
                                  break;
                              }
                         }
                         else if(back == 0)
                         {
                             backCounter = backCounter + 1;
                             if(backCounter==500)
                             {
                                  uiState = restrictedAccess;             //change uiState to restrictedAccess
                                  backCounter = 0;
                                  break;
                             }
                         }
                         else if(enter == 0)
                         {
                             Delay_ms(1);
                             enterCounter = enterCounter + 1;
                             if(enterCounter==500)
                             {
                                 uiState = RatedVoltageSetting;
                                 enteredFirstTime = 0;
                                 enterCounter = 0;
                                 break;
                             }
                         }
                         else
                         {
                            upCounter = 0;
                            downCounter = 0;
                            backCounter = 0;
                            enterCounter = 0;
                         }
                     }
                     break;
                 }

                 case setRatedFrequency:
                 {
                      Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                      Lcd_out(1, 4, "Set Rated");
                      Lcd_out(2, 4, "Frequency?");
                      Delay_ms(500);
                      upCounter = 0;
                      downCounter = 0;
                      enterCounter = 0;
                      backCounter = 0;
                      while(1)
                      {
                         if(back==0)
                         {
                             Delay_ms(1);
                             backCounter= backCounter+ 1;
                             if(backCounter==500)
                             {
                                 uiState = restrictedAccess;
                                 backCounter= 0;
                                 break;
                             } // if end

                         } // if end
                         else if(enter==0)
                         {
                             Delay_ms(1);
                             enterCounter= enterCounter+ 1;
                             if(enterCounter==500)
                             {
                                 uiState = RatedFrequencySetting;
                                 enterCounter= 0;
                                 break;
                             } // if end
                         } // if end
                         else if(up==0)
                         {
                             Delay_ms(1);
                             upCounter= upCounter+ 1;
                             if(upCounter==500)
                             {
                                 uiState = setDateTime;
                                 upCounter= 0;
                                 break;
                             } // if end

                         } // if end
                         else if(down==0)
                         {
                             Delay_ms(1);
                             downCounter= downCounter+ 1;
                             if(downCounter==500)
                             {
                                 uiState = setRatedVoltage;
                                 downCounter= 0;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             upCounter= 0;
                             downCounter= 0;
                             enterCounter = 0;
                             backCounter = 0;
                         }
                      }
                      break;
                  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  ///////////////////////////////////////////////////////////////
                 case RatedVoltageSetting:
                  {

                      A2 = EEPROM_ReadByte(ratedVoltageLocation);
                      A1 = EEPROM_ReadByte(ratedVoltageLocation+1);
                      eepromRatedVoltage= ((Uint16) A1) | ((Uint16) (A2 << 8));

                     while(1)
                     {
                         if(enteredFirstTime == 0)
                         {
                             Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                             Lcd_out(1, 1, "Rated Volt:    V");
                             intToStr(eepromRatedVoltage , eepromRatedVoltageStr, 3);
                             Lcd_out(1, 12, eepromRatedVoltageStr);
                             Lcd_out(2, 1, "Change to: ___ V");
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 12, number1Str);
                             enteredFirstTime = 1;
                             Delay_ms(500);

                         } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setRatedVoltage;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end

                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end

                         if(up==0)
                         {
                             upCounter= upCounter+ 1;
                             Delay_ms(1);

                             if(upCounter== 500)
                             {
                                 number1=number1+1;
                                 if(number1==10)
                                 {
                                     number1=0;
                                 }
                                 upCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end

                         } // if end
                         else
                         {
                             upCounter= 0;
                         } // else end
                         if(down==0)
                         {
                             downCounter= downCounter+ 1;
                             Delay_ms(1);
                             if(downCounter== 500)
                             {
                                 if(number1==0)
                                 {
                                     number1=10;
                                 }
                                 number1=number1-1;
                                 downCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end

                         } // if end
                         else
                         {
                             downCounter= 0;
                         } // else end
                     } // while end

                     while(1)
                     {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                             Lcd_out(1, 1, "Rated Volt:    V");
                             intToStr(eepromRatedVoltage , eepromRatedVoltageStr, 3);
                             Lcd_out(1, 12, eepromRatedVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                        if(back==0)
                        {
                            backCounter= backCounter+ 1;
                            Delay_ms(1);
                            if(backCounter== 500)
                            {
                                uiState = setRatedVoltage;
                                backCounter= 0;
                                backTimer = 1;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                            backCounter= 0;
                        } // else end
                        if(enter==0)
                        {
                            enterCounter= enterCounter+ 1;
                            Delay_ms(1);
                            if(enterCounter== 500)
                            {
                                enterCounter= 0;
                                enteredFirstTime = 0;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                                enterCounter= 0;
                        } // else end
                        if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter == 500)
                            {
                                number2=number2+1;
                                if(number2==10)
                                {
                                   number2=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter == 500)
                            {
                                if(number2==0)
                                {
                                   number2=10;
                                }
                                number2=number2-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                     } //  while end
                     while(1)                          //
                     {
                         if(enteredFirstTime == 0)
                         {
                               Lcd_Cmd(_LCD_CLEAR);
                               Lcd_out(1, 1, "Rated Volt:    V");
                               intToStr(eepromRatedVoltage , eepromRatedVoltageStr, 3);
                               Lcd_out(1, 12, eepromRatedVoltageStr);
                               intToStr(number1, number1Str, 1);
                               Lcd_out(2, 1, "Change to: ___ V");
                               Lcd_out(2, 12, number1Str);
                               intToStr(number2, number2Str, 1);
                               Lcd_out(2, 13, number2Str);
                               intToStr(number3, number3Str, 1);
                               Lcd_out(2, 14, number3Str);
                               Delay_ms(500);
                               enteredFirstTime = 1;
                          } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setRatedVoltage;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(up==0)
                          {
                              upCounter= upCounter+ 1;
                              Delay_ms(1);
                              if(upCounter== 500)
                              {
                                  number3=number3+1;
                                  if(number3 == 10)
                                  {
                                     number3=0;
                                  }
                                  upCounter= 0;
                                  enteredFirstTime = 0;
                              } // if end
                          } // if end
                          else
                          {
                              upCounter= 0;
                          } // else end
                          if(down==0)
                          {
                              downCounter= downCounter+ 1;
                              Delay_ms(1);
                              if(downCounter== 500)
                              {
                                  if(number3 == 0)
                                  {
                                     number3 = 10;
                                  }
                                  number3=number3-1;
                                  downCounter= 0;
                                  enteredFirstTime = 0;
                              } // if end
                          } // if end
                          else
                          {
                              downCounter= 0;
                          } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 RatedVoltage=(number1*100)+(number2*10)+number3;
                                 if(RatedVoltage > 001 && RatedVoltage < 400)
                                 {
                                     intToStr(RatedVoltage, RatedVoltageStr, 3);
                                     A2 = (RatedVoltage>>8);
                                     A1 = (RatedVoltage & 0x00FF);
                                     EEPROM_WriteByte(ratedVoltageLocation, A2);   //rated Voltage location=ratedVoltageLocation+2,+3
                                     EEPROM_WriteByte(ratedVoltageLocation+1, A1);
                                     Lcd_Cmd(_LCD_CLEAR);
                                     Lcd_out(1, 3, "Volts Changed");
                                     Lcd_out(2, 3, "Successfully");
                                     Delay_ms(2000);
                                     uiState = restrictedAccess;           //setRatedVoltage
                                     ratedMotorVoltage = RatedVoltage;
                                 }
                                 else
                                 {
                                    Lcd_Cmd(_LCD_CLEAR);
                                    Lcd_out(1, 3, "Out of Range");
                                    Delay_ms(1500);
                                    uiState = RatedVoltageSetting;
                                 }
                                 number1 =0;
                                 number2 =0;
                                 number3 =0;
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end

                     } //  while end
                     break;

                  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  case RatedFrequencySetting:
                  {
                       B2 = EEPROM_ReadByte(ratedVoltageLocation+2);
                       B1 = EEPROM_ReadByte(ratedVoltageLocation+3);
                       eepromRatedFrequency= ((Uint16) B1) | ((Uint16) (B2 << 8));

                      while(1)
                      {
                          if(enteredFirstTime == 0)
                          {
                              Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                              Lcd_out(1, 1, "Rated Freq:   Hz");
                              intToStr(eepromRatedFrequency , eepromRatedFrequencyStr, 3);
                              Lcd_out(1, 12, eepromRatedFrequencyStr);
                              Lcd_out(2, 1, "Change to: ___Hz");
                              intToStr(number1, number1Str, 1);
                              Lcd_out(2, 12, number1Str);
                              enteredFirstTime = 1;
                              Delay_ms(500);

                          } // if end
                          if(back==0)
                          {
                              backCounter= backCounter+ 1;
                              Delay_ms(1);
                              if(backCounter== 500)
                              {
                                  uiState = setRatedFrequency;
                                  backCounter= 0;
                                  backTimer = 1;
                                  break;
                              } // if end
                          } // if end
                          else
                          {
                              backCounter= 0;
                          } // else end
                          if(enter==0)
                          {
                              enterCounter= enterCounter+ 1;
                              Delay_ms(1);
                              if(enterCounter== 500)
                              {
                                  enterCounter= 0;
                                  enteredFirstTime = 0;
                                  break;
                              } // if end

                          } // if end
                          else
                          {
                              enterCounter= 0;
                          } // else end

                          if(up==0)
                          {
                              upCounter= upCounter+ 1;
                              Delay_ms(1);

                              if(upCounter== 500)
                              {
                                  number1=number1+1;
                                  if(number1==10)
                                  {
                                      number1=0;
                                  }
                                  upCounter= 0;
                                  enteredFirstTime = 0;
                              } // if end

                          } // if end
                          else
                          {
                              upCounter= 0;
                          } // else end
                          if(down==0)
                          {
                              downCounter= downCounter+ 1;
                              Delay_ms(1);
                              if(downCounter== 500)
                              {
                                  if(number1==0)
                                  {
                                      number1=10;
                                  }
                                  number1=number1-1;
                                  downCounter= 0;
                                  enteredFirstTime = 0;
                              } // if end

                          } // if end
                          else
                          {
                              downCounter= 0;
                          } // else end
                      } // while end

                      while(1)
                      {
                        if(enteredFirstTime == 0)
                        {
                              Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                              Lcd_out(1, 1, "Rated Freq:   Hz");
                              intToStr(eepromRatedFrequency , eepromRatedFrequencyStr, 3);
                              Lcd_out(1, 12, eepromRatedFrequencyStr);
                              intToStr(number1, number1Str, 1);
                              Lcd_out(2, 1, "Change to: ___Hz");
                              Lcd_out(2, 12, number1Str);
                              intToStr(number2, number2Str, 1);
                              Lcd_out(2, 13, number2Str);
                              Delay_ms(500);
                              enteredFirstTime = 1;
                         } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setRatedFrequency;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                                 enterCounter= 0;
                         } // else end
                         if(up==0)
                         {
                             upCounter= upCounter+ 1;
                             Delay_ms(1);
                             if(upCounter == 500)
                             {
                                 number2=number2+1;
                                 if(number2==10)
                                 {
                                    number2=0;
                                 }
                                 upCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end
                         } // if end
                         else
                         {
                             upCounter= 0;
                         } // else end
                         if(down==0)
                         {
                             downCounter= downCounter+ 1;
                             Delay_ms(1);
                             if(downCounter == 500)
                             {
                                 if(number2==0)
                                 {
                                    number2=10;
                                 }
                                 number2=number2-1;
                                 downCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end
                         } // if end
                         else
                         {
                             downCounter= 0;
                         } // else end
                      } //  while end
                      while(1)                          //
                      {
                          if(enteredFirstTime == 0)
                          {
                                Lcd_Cmd(_LCD_CLEAR);
                                Lcd_out(1, 1, "Rated Freq:   Hz");
                                intToStr(eepromRatedFrequency , eepromRatedFrequencyStr, 3);
                                Lcd_out(1, 12, eepromRatedFrequencyStr);
                                intToStr(number1, number1Str, 1);
                                Lcd_out(2, 1, "Change to: ___Hz");
                                Lcd_out(2, 12, number1Str);
                                intToStr(number2, number2Str, 1);
                                Lcd_out(2, 13, number2Str);
                                intToStr(number3, number3Str, 1);
                                Lcd_out(2, 14, number3Str);
                                Delay_ms(500);
                                enteredFirstTime = 1;
                           } // if end
                          if(back==0)
                          {
                              backCounter= backCounter+ 1;
                              Delay_ms(1);
                              if(backCounter== 500)
                              {
                                  uiState = setRatedFrequency;
                                  backCounter= 0;
                                  backTimer = 1;
                                  break;
                              } // if end
                          } // if end
                          else
                          {
                              backCounter= 0;
                          } // else end
                          if(up==0)
                           {
                               upCounter= upCounter+ 1;
                               Delay_ms(1);
                               if(upCounter== 500)
                               {
                                   number3=number3+1;
                                   if(number3 == 10)
                                   {
                                      number3=0;
                                   }
                                   upCounter= 0;
                                   enteredFirstTime = 0;
                               } // if end
                           } // if end
                           else
                           {
                               upCounter= 0;
                           } // else end
                           if(down==0)
                           {
                               downCounter= downCounter+ 1;
                               Delay_ms(1);
                               if(downCounter== 500)
                               {
                                   if(number3 == 0)
                                   {
                                      number3 = 10;
                                   }
                                   number3=number3-1;
                                   uiState= dateTimeCheck;
                                   downCounter= 0;
                                   enteredFirstTime = 0;
                               } // if end
                           } // if end
                           else
                           {
                               downCounter= 0;
                           } // else end
                          if(enter==0)
                          {
                              enterCounter= enterCounter+ 1;
                              Delay_ms(1);
                              if(enterCounter== 500)
                              {
                                  RatedFrequency=(number1*100)+(number2*10)+number3;
                                  if(RatedFrequency > 001 && RatedFrequency < 400)
                                  {
                                      intToStr(RatedFrequency, RatedFrequencyStr, 3);
                                      B2 = (RatedFrequency>>8);
                                      B1 = (RatedFrequency & 0x00FF);
                                      EEPROM_WriteByte(ratedVoltageLocation+2, B2);   //rated frequency location=ratedVoltageLocation+2,+3
                                      EEPROM_WriteByte(ratedVoltageLocation+3, B1);
                                      Lcd_Cmd(_LCD_CLEAR);
                                      Lcd_out(1, 3, "Freq Changed");
                                      Lcd_out(2, 3, "Successfully");
                                      Delay_ms(2000);
                                      uiState = setgridUVTripVoltage;
                                      ratedMotorFrequency = RatedFrequency;
                                  }
                                  else
                                  {
                                      Lcd_Cmd(_LCD_CLEAR);
                                      Lcd_out(1, 3, "Out of Range");
                                      Delay_ms(1500);
                                      uiState = RatedFrequencySetting;
                                  }
                                  number1 =0;
                                  number2 =0;
                                  number3 =0;
                                  enterCounter= 0;
                                  enteredFirstTime = 0;
                                  break;
                              } // if end
                          } // if end
                          else
                          {
                              enterCounter= 0;
                          } // else end

                      } //  while end
                      break;
                  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  case setgridUVTripVoltage:
                   {
                        Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                        Lcd_out(1, 1, "Set gridUVTrip");
                        Lcd_out(2, 4, "Voltage?");
                        Delay_ms(500);
                        upCounter = 0;
                        downCounter = 0;
                        enterCounter = 0;
                        backCounter = 0;
                        while(1)
                        {
                           if(back==0)
                           {
                               Delay_ms(1);
                               backCounter= backCounter+ 1;
                               if(backCounter==500)
                               {
                                   uiState = restrictedAccess;
                                   backCounter= 0;
                                   break;
                               } // if end

                           } // if end
                           else if(enter==0)
                           {
                               Delay_ms(1);
                               enterCounter= enterCounter+ 1;
                               if(enterCounter==500)
                               {
                                   uiState = gridUVTripVoltageSetting;
                                   enterCounter= 0;
                                   break;
                               } // if end
                           } // if end
                           else if(up==0)
                           {
                               Delay_ms(1);
                               upCounter= upCounter+ 1;
                               if(upCounter==500)
                               {
                                   uiState = setRatedVoltage;
                                   upCounter= 0;
                                   break;
                               } // if end

                           } // if end
                           else if(down==0)
                           {
                               Delay_ms(1);
                               downCounter= downCounter+ 1;
                               if(downCounter==500)
                               {
                                   uiState = setgridLowerStartVoltage;
                                   downCounter= 0;
                                   break;
                               } // if end
                           } // if end
                           else
                           {
                               upCounter= 0;
                               downCounter= 0;
                               enterCounter = 0;
                               backCounter = 0;
                           }
                        }
                        break;
                    }
                  case gridUVTripVoltageSetting:
                  {

                      C2 = EEPROM_ReadByte(ratedVoltageLocation+4);
                      C1 = EEPROM_ReadByte(ratedVoltageLocation+5);
                      eepromGridUVTripVoltage= ((Uint16) C1) | ((Uint16) (C2 << 8));

                     while(1)
                     {
                         if(enteredFirstTime == 0)
                         {
                             Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                             Lcd_out(1, 1, "GridUVTrip:    V");
                             intToStr(eepromGridUVTripVoltage , eepromGridUVTripVoltageStr, 3);
                             Lcd_out(1, 12, eepromGridUVTripVoltageStr);
                             Lcd_out(2, 1, "Change to: ___ V");
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 12, number1Str);
                             enteredFirstTime = 1;
                             Delay_ms(500);

                         } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setgridUVTripVoltage;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end

                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end

                         if(up==0)
                         {
                             upCounter= upCounter+ 1;
                             Delay_ms(1);

                             if(upCounter== 500)
                             {
                                 number1=number1+1;
                                 if(number1==10)
                                 {
                                     number1=0;
                                 }
                                 upCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end

                         } // if end
                         else
                         {
                             upCounter= 0;
                         } // else end
                         if(down==0)
                         {
                             downCounter= downCounter+ 1;
                             Delay_ms(1);
                             if(downCounter== 500)
                             {
                                 if(number1==0)
                                 {
                                     number1=10;
                                 }
                                 number1=number1-1;
                                 downCounter= 0;
                                 enteredFirstTime = 0;
                             } // if end

                         } // if end
                         else
                         {
                             downCounter= 0;
                         } // else end
                     } // while end

                     while(1)
                     {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                             Lcd_out(1, 1, "GridUVTrip:    V");
                             intToStr(eepromGridUVTripVoltage , eepromGridUVTripVoltageStr, 3);
                             Lcd_out(1, 12, eepromGridUVTripVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                        if(back==0)
                        {
                            backCounter= backCounter+ 1;
                            Delay_ms(1);
                            if(backCounter== 500)
                            {
                                uiState = setgridUVTripVoltage;
                                backCounter= 0;
                                backTimer = 1;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                            backCounter= 0;
                        } // else end
                        if(enter==0)
                        {
                            enterCounter= enterCounter+ 1;
                            Delay_ms(1);
                            if(enterCounter== 500)
                            {
                                enterCounter= 0;
                                enteredFirstTime = 0;
                                break;
                            } // if end
                        } // if end
                        else
                        {
                                enterCounter= 0;
                        } // else end
                        if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter == 500)
                            {
                                number2=number2+1;
                                if(number2==10)
                                {
                                   number2=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter == 500)
                            {
                                if(number2==0)
                                {
                                   number2=10;
                                }
                                number2=number2-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                     } //  while end
                     while(1)                          //
                     {
                         if(enteredFirstTime == 0)
                         {
                               Lcd_Cmd(_LCD_CLEAR);
                               Lcd_out(1, 1, "GridUVTrip:    V");
                               intToStr(eepromGridUVTripVoltage , eepromGridUVTripVoltageStr, 3);
                               Lcd_out(1, 12, eepromGridUVTripVoltageStr);
                               intToStr(number1, number1Str, 1);
                               Lcd_out(2, 1, "Change to: ___ V");
                               Lcd_out(2, 12, number1Str);
                               intToStr(number2, number2Str, 1);
                               Lcd_out(2, 13, number2Str);
                               intToStr(number3, number3Str, 1);
                               Lcd_out(2, 14, number3Str);
                               Delay_ms(500);
                               enteredFirstTime = 1;
                          } // if end
                         if(back==0)
                         {
                             backCounter= backCounter+ 1;
                             Delay_ms(1);
                             if(backCounter== 500)
                             {
                                 uiState = setgridUVTripVoltage;
                                 backCounter= 0;
                                 backTimer = 1;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             backCounter= 0;
                         } // else end
                         if(up==0)
                          {
                              upCounter= upCounter+ 1;
                              Delay_ms(1);
                              if(upCounter== 500)
                              {
                                  number3=number3+1;
                                  if(number3 == 10)
                                  {
                                     number3=0;
                                  }
                                  upCounter= 0;
                                  enteredFirstTime = 0;
                              } // if end
                          } // if end
                          else
                          {
                              upCounter= 0;
                          } // else end
                          if(down==0)
                          {
                              downCounter= downCounter+ 1;
                              Delay_ms(1);
                              if(downCounter== 500)
                              {
                                  if(number3 == 0)
                                  {
                                     number3 = 10;
                                  }
                                  number3=number3-1;
                                  downCounter= 0;
                                  enteredFirstTime = 0;
                              } // if end
                          } // if end
                          else
                          {
                              downCounter= 0;
                          } // else end
                         if(enter==0)
                         {
                             enterCounter= enterCounter+ 1;
                             Delay_ms(1);
                             if(enterCounter== 500)
                             {
                                 gridUVTripVoltageInt=(number1*100)+(number2*10)+number3;
                                 if(gridUVTripVoltageInt > 001 && gridUVTripVoltageInt < 400)
                                 {
                                     intToStr(gridUVTripVoltageInt, gridUVTripVoltageIntStr, 3);
                                     C2 = (gridUVTripVoltageInt>>8);
                                     C1 = (gridUVTripVoltageInt & 0x00FF);
                                     EEPROM_WriteByte(ratedVoltageLocation+4, C2);   //rated frequency location=ratedVoltageLocation+2,+3
                                     EEPROM_WriteByte(ratedVoltageLocation+5, C1);
                                     Lcd_Cmd(_LCD_CLEAR);
                                     Lcd_out(1, 3, "gridUVTr Changed");
                                     Lcd_out(2, 3, "Successfully");
                                     Delay_ms(2000);
                                     uiState = setgridLowerStartVoltage;
                                     gridUVTripVoltage = gridUVTripVoltageInt;
                                 }
                                 else
                                 {
                                     Lcd_Cmd(_LCD_CLEAR);
                                     Lcd_out(1, 3, "Out of Range");
                                     Delay_ms(1500);
                                     uiState = gridUVTripVoltageSetting;
                                 }
                                 number1 =0;
                                 number2 =0;
                                 number3 =0;
                                 enterCounter= 0;
                                 enteredFirstTime = 0;
                                 break;
                             } // if end
                         } // if end
                         else
                         {
                             enterCounter= 0;
                         } // else end

                     } //  while end
                     break;
                  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                case setgridLowerStartVoltage:
                 {
                    Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                    Lcd_out(1, 1, "Set gridLower");
                    Lcd_out(2, 4, "StartVoltage?");
                    Delay_ms(500);
                    upCounter = 0;
                    downCounter = 0;
                    enterCounter = 0;
                    backCounter = 0;
                    while(1)
                    {
                       if(back==0)
                       {
                           Delay_ms(1);
                           backCounter= backCounter+ 1;
                           if(backCounter==500)
                           {
                               uiState = restrictedAccess;
                               backCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(enter==0)
                       {
                           Delay_ms(1);
                           enterCounter= enterCounter+ 1;
                           if(enterCounter==500)
                           {
                               uiState = gridLowerStartVoltageSetting;
                               enterCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else if(up==0)
                       {
                           Delay_ms(1);
                           upCounter= upCounter+ 1;
                           if(upCounter==500)
                           {
                               uiState = setgridUVTripVoltage;
                               upCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(down==0)
                       {
                           Delay_ms(1);
                           downCounter= downCounter+ 1;
                           if(downCounter==500)
                           {
                               uiState = setgridUpperStartVoltage;
                               downCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                           downCounter= 0;
                           enterCounter = 0;
                           backCounter = 0;
                       }
                    }
                    break;
                }


                case gridLowerStartVoltageSetting:
                {
                    D2 = EEPROM_ReadByte(ratedVoltageLocation+6);
                    D1 = EEPROM_ReadByte(ratedVoltageLocation+7);
                    eepromRatedFrequency= ((Uint16) D1) | ((Uint16) (D2 << 8));

                   while(1)
                   {
                       if(enteredFirstTime == 0)
                       {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "GridLowStV:    V");
                           intToStr(eepromGridLowerStartVoltage , eepromGridLowerStartVoltageStr, 3);
                           Lcd_out(1, 13, eepromGridLowerStartVoltageStr);
                           Lcd_out(2, 1, "Change to: ___V");
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, number1Str);
                           enteredFirstTime = 1;
                           Delay_ms(500);

                       } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setgridLowerStartVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end

                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                       if(up==0)
                       {
                           upCounter= upCounter+ 1;
                           Delay_ms(1);

                           if(upCounter== 500)
                           {
                               number1=number1+1;
                               if(number1==10)
                               {
                                   number1=0;
                               }
                               upCounter= 0;
                               enteredFirstTime = 0;
                           } // if end

                       } // if end
                       else
                       {
                           upCounter= 0;
                       } // else end
                       if(down==0)
                       {
                           downCounter= downCounter+ 1;
                           Delay_ms(1);
                           if(downCounter== 500)
                           {
                               if(number1==0)
                               {
                                   number1=10;
                               }
                               number1=number1-1;
                               downCounter= 0;
                               enteredFirstTime = 0;
                           } // if end

                       } // if end
                       else
                       {
                           downCounter= 0;
                       } // else end
                   } // while end

                   while(1)
                   {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "GridLowStV:    V");
                           intToStr(eepromGridLowerStartVoltage , eepromGridLowerStartVoltageStr, 3);
                           Lcd_out(1, 12, eepromGridLowerStartVoltageStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                      if(back==0)
                      {
                          backCounter= backCounter+ 1;
                          Delay_ms(1);
                          if(backCounter== 500)
                          {
                              uiState = setgridLowerStartVoltage;
                              backCounter= 0;
                              backTimer = 1;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          backCounter= 0;
                      } // else end
                      if(enter==0)
                      {
                          enterCounter= enterCounter+ 1;
                          Delay_ms(1);
                          if(enterCounter== 500)
                          {
                              enterCounter= 0;
                              enteredFirstTime = 0;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                              enterCounter= 0;
                      } // else end
                      if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter == 500)
                          {
                              number2=number2+1;
                              if(number2==10)
                              {
                                 number2=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter == 500)
                          {
                              if(number2==0)
                              {
                                 number2=10;
                              }
                              number2=number2-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                   } //  while end
                   while(1)                          //
                   {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);
                             Lcd_out(1, 1, "GridLowStV:    V");
                             intToStr(eepromGridLowerStartVoltage , eepromGridLowerStartVoltageStr, 3);
                             Lcd_out(1, 12, eepromGridLowerStartVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             intToStr(number3, number3Str, 1);
                             Lcd_out(2, 14, number3Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setgridLowerStartVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                number3=number3+1;
                                if(number3 == 10)
                                {
                                   number3=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(number3 == 0)
                                {
                                   number3 = 10;
                                }
                                number3=number3-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               gridLowerStartVoltageInt=(number1*100)+(number2*10)+number3;
                               if(gridLowerStartVoltageInt > 010 && gridLowerStartVoltageInt < 450)
                               {
                                   intToStr(gridLowerStartVoltageInt, gridLowerStartVoltageIntStr, 3);
                                   D2 = (gridLowerStartVoltageInt>>8);
                                   D1 = (gridLowerStartVoltageInt & 0x00FF);
                                   EEPROM_WriteByte(ratedVoltageLocation+6, D2);
                                   EEPROM_WriteByte(ratedVoltageLocation+7, D1);
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 2, "Grid Low Volt");
                                   Lcd_out(2, 2, "change Success");
                                   Delay_ms(2000);
                                   uiState = setgridUpperStartVoltage;
                                   gridLowerStartVoltage = gridLowerStartVoltageInt;
                               }
                               else
                               {
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "Out of Range");
                                   Delay_ms(1500);
                                   uiState = gridLowerStartVoltageSetting;
                               }
                               number1 =0;
                               number2 =0;
                               number3 =0;
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                   } //  while end
                   break;
                }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

             case setgridUpperStartVoltage:
              {

                  Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                  Lcd_out(1, 1, "Set gridUpper");
                  Lcd_out(2, 4, "StartVoltage?");
                  Delay_ms(500);
                  upCounter = 0;
                  downCounter = 0;
                  enterCounter = 0;
                  backCounter = 0;
                  while(1)
                  {
                     if(back==0)
                     {
                         Delay_ms(1);
                         backCounter= backCounter+ 1;
                         if(backCounter==500)
                         {
                             uiState = restrictedAccess;
                             backCounter= 0;
                             break;
                         } // if end

                     } // if end
                     else if(enter==0)
                     {
                         Delay_ms(1);
                         enterCounter= enterCounter+ 1;
                         if(enterCounter==500)
                         {
                             uiState = gridUpperStartVoltageSetting;
                             enterCounter= 0;
                             break;
                         } // if end
                     } // if end
                     else if(up==0)
                     {
                         Delay_ms(1);
                         upCounter= upCounter+ 1;
                         if(upCounter==500)
                         {
                             uiState = setgridLowerStartVoltage;
                             upCounter= 0;
                             break;
                         } // if end

                     } // if end
                     else if(down==0)
                     {
                         Delay_ms(1);
                         downCounter= downCounter+ 1;
                         if(downCounter==500)
                         {
                             uiState = setgridOVTripVoltage;
                             downCounter= 0;
                             break;
                         } // if end
                     } // if end
                     else
                     {
                         upCounter= 0;
                         downCounter= 0;
                         enterCounter = 0;
                         backCounter = 0;
                     }
                  }
                  break;
              }

                case gridUpperStartVoltageSetting:
              {
                  E2 = EEPROM_ReadByte(ratedVoltageLocation+8);
                  E1 = EEPROM_ReadByte(ratedVoltageLocation+9);
                  eepromGridUVTripVoltage= ((Uint16) E1) | ((Uint16) (E2 << 8));

                 while(1)
                 {
                     if(enteredFirstTime == 0)
                     {
                         Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                         Lcd_out(1, 1, "GridUpperStart:  V");
                         intToStr(eepromGridUpperStartVoltage , eepromGridUpperStartVoltageStr, 3);
                         Lcd_out(1, 12, eepromGridUpperStartVoltageStr);
                         Lcd_out(2, 1, "Change to: ___ V");
                         intToStr(number1, number1Str, 1);
                         Lcd_out(2, 12, number1Str);
                         enteredFirstTime = 1;
                         Delay_ms(500);

                     } // if end
                     if(back==0)
                     {
                         backCounter= backCounter+ 1;
                         Delay_ms(1);
                         if(backCounter== 500)
                         {
                             uiState = setgridUpperStartVoltage;
                             backCounter= 0;
                             backTimer = 1;
                             break;
                         } // if end
                     } // if end
                     else
                     {
                         backCounter= 0;
                     } // else end
                     if(enter==0)
                     {
                         enterCounter= enterCounter+ 1;
                         Delay_ms(1);
                         if(enterCounter== 500)
                         {
                             enterCounter= 0;
                             enteredFirstTime = 0;
                             break;
                         } // if end

                     } // if end
                     else
                     {
                         enterCounter= 0;
                     } // else end

                     if(up==0)
                     {
                         upCounter= upCounter+ 1;
                         Delay_ms(1);

                         if(upCounter== 500)
                         {
                             number1=number1+1;
                             if(number1==10)
                             {
                                 number1=0;
                             }
                             upCounter= 0;
                             enteredFirstTime = 0;
                         } // if end

                     } // if end
                     else
                     {
                         upCounter= 0;
                     } // else end
                     if(down==0)
                     {
                         downCounter= downCounter+ 1;
                         Delay_ms(1);
                         if(downCounter== 500)
                         {
                             if(number1==0)
                             {
                                 number1=10;
                             }
                             number1=number1-1;
                             downCounter= 0;
                             enteredFirstTime = 0;
                         } // if end

                     } // if end
                     else
                     {
                         downCounter= 0;
                     } // else end
                 } // while end

                 while(1)
                 {
                   if(enteredFirstTime == 0)
                   {
                         Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                         Lcd_out(1, 1, "GridUpperStart:    V");
                         intToStr(eepromGridUpperStartVoltage , eepromGridUpperStartVoltageStr, 3);
                         Lcd_out(1, 12, eepromGridUpperStartVoltageStr);
                         intToStr(number1, number1Str, 1);
                         Lcd_out(2, 1, "Change to: ___ V");
                         Lcd_out(2, 12, number1Str);
                         intToStr(number2, number2Str, 1);
                         Lcd_out(2, 13, number2Str);
                         Delay_ms(500);
                         enteredFirstTime = 1;
                    } // if end
                    if(back==0)
                    {
                        backCounter= backCounter+ 1;
                        Delay_ms(1);
                        if(backCounter== 500)
                        {
                            uiState = setgridUpperStartVoltage;
                            backCounter= 0;
                            backTimer = 1;
                            break;
                        } // if end
                    } // if end
                    else
                    {
                        backCounter= 0;
                    } // else end
                    if(enter==0)
                    {
                        enterCounter= enterCounter+ 1;
                        Delay_ms(1);
                        if(enterCounter== 500)
                        {
                            enterCounter= 0;
                            enteredFirstTime = 0;
                            break;
                        } // if end
                    } // if end
                    else
                    {
                            enterCounter= 0;
                    } // else end
                    if(up==0)
                    {
                        upCounter= upCounter+ 1;
                        Delay_ms(1);
                        if(upCounter == 500)
                        {
                            number2=number2+1;
                            if(number2==10)
                            {
                               number2=0;
                            }
                            upCounter= 0;
                            enteredFirstTime = 0;
                        } // if end
                    } // if end
                    else
                    {
                        upCounter= 0;
                    } // else end
                    if(down==0)
                    {
                        downCounter= downCounter+ 1;
                        Delay_ms(1);
                        if(downCounter == 500)
                        {
                            if(number2==0)
                            {
                               number2=10;
                            }
                            number2=number2-1;
                            downCounter= 0;
                            enteredFirstTime = 0;
                        } // if end
                    } // if end
                    else
                    {
                        downCounter= 0;
                    } // else end
                 } //  while end
                 while(1)                          //
                 {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);
                           Lcd_out(1, 1, "GridUpperStart:    V");
                           intToStr(eepromGridUpperStartVoltage , eepromGridUpperStartVoltageStr, 3);
                           Lcd_out(1, 12, eepromGridUpperStartVoltageStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___ V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           intToStr(number3, number3Str, 1);
                           Lcd_out(2, 14, number3Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                     if(back==0)
                     {
                         backCounter= backCounter+ 1;
                         Delay_ms(1);
                         if(backCounter== 500)
                         {
                             uiState = setgridUpperStartVoltage;
                             backCounter= 0;
                             backTimer = 1;
                             break;
                         } // if end
                     } // if end
                     else
                     {
                         backCounter= 0;
                     } // else end
                     if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter== 500)
                          {
                              number3=number3+1;
                              if(number3 == 10)
                              {
                                 number3=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter== 500)
                          {
                              if(number3 == 0)
                              {
                                 number3 = 10;
                              }
                              number3=number3-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                     if(enter==0)
                     {
                         enterCounter= enterCounter+ 1;
                         Delay_ms(1);
                         if(enterCounter== 500)
                         {
                             gridUpperStartVoltageInt=(number1*100)+(number2*10)+number3;
                             if(gridUpperStartVoltageInt > 010 && gridUpperStartVoltageInt < 500)
                             {
                                 intToStr(gridUpperStartVoltageInt, gridUpperStartVoltageIntStr, 3);
                                 E2 = (gridUpperStartVoltageInt>>8);
                                 E1 = (gridUpperStartVoltageInt & 0x00FF);
                                 EEPROM_WriteByte(ratedVoltageLocation+8, E2);
                                 EEPROM_WriteByte(ratedVoltageLocation+9, E1);
                                 Lcd_Cmd(_LCD_CLEAR);
                                 Lcd_out(1, 3, "GridUpperStartChanged");
                                 Lcd_out(2, 3, "Successfully");
                                 Delay_ms(2000);
                                 uiState = setgridOVTripVoltage;
                                 gridUpperStartVoltage = gridUpperStartVoltageInt;
                             }
                             else
                             {
                                 Lcd_Cmd(_LCD_CLEAR);
                                 Lcd_out(1, 3, "Out of Range");
                                 Delay_ms(1500);
                                 uiState = setgridUpperStartVoltage;
                             }
                             number1 =0;
                             number2 =0;
                             number3 =0;
                             enterCounter= 0;
                             enteredFirstTime = 0;
                             break;
                         } // if end
                     } // if end
                     else
                     {
                         enterCounter= 0;
                     } // else end

                 } //  while end
                 break;
              }
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                case setgridOVTripVoltage:
                {
                    Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                    Lcd_out(1, 1, "Set grid OV");
                    Lcd_out(2, 4, "Trip Voltage?");
                    Delay_ms(500);
                    upCounter = 0;
                    downCounter = 0;
                    enterCounter = 0;
                    backCounter = 0;
                    while(1)
                    {
                       if(back==0)
                       {
                           Delay_ms(1);
                           backCounter= backCounter+ 1;
                           if(backCounter==500)
                           {
                               uiState = restrictedAccess;
                               backCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(enter==0)
                       {
                           Delay_ms(1);
                           enterCounter= enterCounter+ 1;
                           if(enterCounter==500)
                           {
                               uiState = gridOVTripVoltageSetting;
                               enterCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else if(up==0)
                       {
                           Delay_ms(1);
                           upCounter= upCounter+ 1;
                           if(upCounter==500)
                           {
                               uiState = setgridUpperStartVoltage;
                               upCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(down==0)
                       {
                           Delay_ms(1);
                           downCounter= downCounter+ 1;
                           if(downCounter==500)
                           {
                               uiState = setdcLinkOVTripVoltage;
                               downCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                           downCounter= 0;
                           enterCounter = 0;
                           backCounter = 0;
                       }
                    }
                    break;
                }

                case gridOVTripVoltageSetting:
                {
                    F2 = EEPROM_ReadByte(ratedVoltageLocation+10);
                    F1 = EEPROM_ReadByte(ratedVoltageLocation+11);
                    eepromGridOVTripVoltage = ((Uint16) F1) | ((Uint16) (F2 << 8));

                   while(1)
                   {
                       if(enteredFirstTime == 0)
                       {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "GridOVTripVolt:  V");
                           intToStr(eepromGridOVTripVoltage , eepromGridOVTripVoltageStr, 3);
                           Lcd_out(1, 12, eepromGridOVTripVoltageStr);
                           Lcd_out(2, 1, "Change to: ___ V");
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 12, number1Str);
                           enteredFirstTime = 1;
                           Delay_ms(500);

                       } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setgridOVTripVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end

                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                       if(up==0)
                       {
                           upCounter= upCounter+ 1;
                           Delay_ms(1);

                           if(upCounter== 500)
                           {
                               number1=number1+1;
                               if(number1==10)
                               {
                                   number1=0;
                               }
                               upCounter= 0;
                               enteredFirstTime = 0;
                           } // if end

                       } // if end
                       else
                       {
                           upCounter= 0;
                       } // else end
                       if(down==0)
                       {
                           downCounter= downCounter+ 1;
                           Delay_ms(1);
                           if(downCounter== 500)
                           {
                               if(number1==0)
                               {
                                   number1=10;
                               }
                               number1=number1-1;
                               downCounter= 0;
                               enteredFirstTime = 0;
                           } // if end

                       } // if end
                       else
                       {
                           downCounter= 0;
                       } // else end
                   } // while end

                   while(1)
                   {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "GridOVTripVolt:    V");
                           intToStr(eepromGridOVTripVoltage , eepromGridOVTripVoltageStr, 3);
                           Lcd_out(1, 12, eepromGridOVTripVoltageStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___ V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                      if(back==0)
                      {
                          backCounter= backCounter+ 1;
                          Delay_ms(1);
                          if(backCounter== 500)
                          {
                              uiState = setgridOVTripVoltage;
                              backCounter= 0;
                              backTimer = 1;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          backCounter= 0;
                      } // else end
                      if(enter==0)
                      {
                          enterCounter= enterCounter+ 1;
                          Delay_ms(1);
                          if(enterCounter== 500)
                          {
                              enterCounter= 0;
                              enteredFirstTime = 0;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                              enterCounter= 0;
                      } // else end
                      if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter == 500)
                          {
                              number2=number2+1;
                              if(number2==10)
                              {
                                 number2=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter == 500)
                          {
                              if(number2==0)
                              {
                                 number2=10;
                              }
                              number2=number2-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                   } //  while end
                   while(1)                          //
                   {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);
                             Lcd_out(1, 1, "GridOVTripVolt:    V");
                             intToStr(eepromGridOVTripVoltage , eepromGridOVTripVoltageStr, 3);
                             Lcd_out(1, 12, eepromGridOVTripVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             intToStr(number3, number3Str, 1);
                             Lcd_out(2, 14, number3Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setgridOVTripVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                number3=number3+1;
                                if(number3 == 10)
                                {
                                   number3=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(number3 == 0)
                                {
                                   number3 = 10;
                                }
                                number3=number3-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               gridOVTripVoltageInt=(number1*100)+(number2*10)+number3;
                               if(gridOVTripVoltageInt > 010 && gridOVTripVoltageInt < 500)
                               {
                                   intToStr(gridOVTripVoltageInt, gridOVTripVoltageIntStr, 3);
                                   F2 = (gridOVTripVoltageInt>>8);
                                   F1 = (gridOVTripVoltageInt & 0x00FF);
                                   EEPROM_WriteByte(ratedVoltageLocation+10, F2);   //rated frequency location=ratedVoltageLocation+2,+3
                                   EEPROM_WriteByte(ratedVoltageLocation+11, F1);
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "GridOVTripVoltChanged");
                                   Lcd_out(2, 3, "Successfully");
                                   Delay_ms(2000);
                                   uiState = setdcLinkOVTripVoltage;
                                   gridOVTripVoltage = gridOVTripVoltageInt;
                               }
                               else
                               {
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "Out of Range");
                                   Delay_ms(1500);
                                   uiState = gridOVTripVoltageSetting;
                               }
                               number1 =0;
                               number2 =0;
                               number3 =0;
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                   } //  while end
                   break;
                }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                case setdcLinkOVTripVoltage:
                {
                    Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                    Lcd_out(1, 1, "Set dcLinkOV");
                    Lcd_out(2, 4, "Trip Voltage?");
                    Delay_ms(500);
                    upCounter = 0;
                    downCounter = 0;
                    enterCounter = 0;
                    backCounter = 0;
                    while(1)
                    {
                       if(back==0)
                       {
                           Delay_ms(1);
                           backCounter= backCounter+ 1;
                           if(backCounter==500)
                           {
                               uiState = restrictedAccess;
                               backCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(enter==0)
                       {
                           Delay_ms(1);
                           enterCounter= enterCounter+ 1;
                           if(enterCounter==500)
                           {
                               uiState = dcLinkOVTripVoltageSetting;
                               enterCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else if(up==0)
                       {
                           Delay_ms(1);
                           upCounter= upCounter+ 1;
                           if(upCounter==500)
                           {
                               uiState = gridOVTripVoltageSetting;
                               upCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(down==0)
                       {
                           Delay_ms(1);
                           downCounter= downCounter+ 1;
                           if(downCounter==500)
                           {
                               uiState = setdcLinkUVTripVoltage;
                               downCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                           downCounter= 0;
                           enterCounter = 0;
                           backCounter = 0;
                       }
                    }
                    break;
                }


                case dcLinkOVTripVoltageSetting:
                {
                    G2 = EEPROM_ReadByte(ratedVoltageLocation+12);
                    G1 = EEPROM_ReadByte(ratedVoltageLocation+13);
                    eepromDCLinkOVTripVoltage= ((Uint16) G1) | ((Uint16) (G2 << 8));

                   while(1)
                   {
                       if(enteredFirstTime == 0)
                       {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "dcLinkOVTripVolt:  V");
                           intToStr(eepromDCLinkOVTripVoltage , eepromDCLinkOVTripVoltageStr, 3);
                           Lcd_out(1, 12, eepromDCLinkOVTripVoltageStr);
                           Lcd_out(2, 1, "Change to: ___ V");
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 12, number1Str);
                           enteredFirstTime = 1;
                           Delay_ms(500);
                       } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setdcLinkOVTripVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                       if(up==0)
                       {
                           upCounter= upCounter+ 1;
                           Delay_ms(1);

                           if(upCounter== 500)
                           {
                               number1=number1+1;
                               if(number1==10)
                               {
                                   number1=0;
                               }
                               upCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                       } // else end
                       if(down==0)
                       {
                           downCounter= downCounter+ 1;
                           Delay_ms(1);
                           if(downCounter== 500)
                           {
                               if(number1==0)
                               {
                                   number1=10;
                               }
                               number1=number1-1;
                               downCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           downCounter= 0;
                       } // else end
                   } // while end

                   while(1)
                   {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "dcLinkOVTripVolt:    V");
                           intToStr(eepromDCLinkOVTripVoltage , eepromDCLinkOVTripVoltageStr, 3);
                           Lcd_out(1, 12, eepromDCLinkOVTripVoltageStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___ V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                      if(back==0)
                      {
                          backCounter= backCounter+ 1;
                          Delay_ms(1);
                          if(backCounter== 500)
                          {
                              uiState = setdcLinkOVTripVoltage;
                              backCounter= 0;
                              backTimer = 1;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          backCounter= 0;
                      } // else end
                      if(enter==0)
                      {
                          enterCounter= enterCounter+ 1;
                          Delay_ms(1);
                          if(enterCounter== 500)
                          {
                              enterCounter= 0;
                              enteredFirstTime = 0;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          enterCounter= 0;
                      } // else end
                      if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter == 500)
                          {
                              number2=number2+1;
                              if(number2==10)
                              {
                                 number2=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter == 500)
                          {
                              if(number2==0)
                              {
                                 number2=10;
                              }
                              number2=number2-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                   } //  while end
                   while(1)                          //
                   {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);
                             Lcd_out(1, 1, "dcLinkOVTripVolt:    V");
                             intToStr(eepromDCLinkOVTripVoltage , eepromDCLinkOVTripVoltageStr, 3);
                             Lcd_out(1, 12, eepromDCLinkOVTripVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             intToStr(number3, number3Str, 1);
                             Lcd_out(2, 14, number3Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setdcLinkOVTripVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                number3=number3+1;
                                if(number3 == 10)
                                {
                                   number3=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(number3 == 0)
                                {
                                   number3 = 10;
                                }
                                number3=number3-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               dcLinkOVTripVoltageInt=(number1*100)+(number2*10)+number3;
                               if(dcLinkOVTripVoltageInt > 010 && dcLinkOVTripVoltageInt < 800)
                               {
                                   intToStr(dcLinkOVTripVoltageInt, dcLinkOVTripVoltageIntStr, 3);
                                   G2 = (dcLinkOVTripVoltageInt>>8);
                                   G1 = (dcLinkOVTripVoltageInt & 0x00FF);
                                   EEPROM_WriteByte(ratedVoltageLocation+12, G2);
                                   EEPROM_WriteByte(ratedVoltageLocation+13, G1);
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "dcLinkOVTripVoltChanged");
                                   Lcd_out(2, 3, "Successfully");
                                   Delay_ms(2000);
                                   uiState = setdcLinkUVTripVoltage;
                                   dcLinkOVTripVoltage = dcLinkOVTripVoltageInt;
                               }
                               else
                               {
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "Out of Range");
                                   Delay_ms(1500);
                                   uiState = dcLinkOVTripVoltageSetting;
                               }
                               number1 =0;
                               number2 =0;
                               number3 =0;
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                   } //  while end
                   break;
                }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////
                case setdcLinkUVTripVoltage:
                {
                    Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                    Lcd_out(1, 1, "Set dcLinkUV");
                    Lcd_out(2, 4, "Trip Voltage?");
                    Delay_ms(500);
                    upCounter = 0;
                    downCounter = 0;
                    enterCounter = 0;
                    backCounter = 0;
                    while(1)
                    {
                       if(back==0)
                       {
                           Delay_ms(1);
                           backCounter= backCounter+ 1;
                           if(backCounter==500)
                           {
                               uiState = restrictedAccess;
                               backCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(enter==0)
                       {
                           Delay_ms(1);
                           enterCounter= enterCounter+ 1;
                           if(enterCounter==500)
                           {
                               uiState = dcLinkUVTripVoltageSetting;
                               enterCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else if(up==0)
                       {
                           Delay_ms(1);
                           upCounter= upCounter+ 1;
                           if(upCounter==500)
                           {
                               uiState = setdcLinkOVTripVoltage;
                               upCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(down==0)
                       {
                           Delay_ms(1);
                           downCounter= downCounter+ 1;
                           if(downCounter==500)
                           {
                               uiState = setdcLinkUpperStartVoltage;
                               downCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                           downCounter= 0;
                           enterCounter = 0;
                           backCounter = 0;
                       }
                    }
                    break;
                }

                case dcLinkUVTripVoltageSetting:
                {
                    H2 = EEPROM_ReadByte(ratedVoltageLocation+14);
                    H1 = EEPROM_ReadByte(ratedVoltageLocation+15);
                    eepromDCLinkUVTripVoltage= ((Uint16) G1) | ((Uint16) (G2 << 8));

                   while(1)
                   {
                       if(enteredFirstTime == 0)
                       {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "dcLinkUVTripVolt:  V");
                           intToStr(eepromDCLinkUVTripVoltage , eepromDCLinkUVTripVoltageStr, 3);
                           Lcd_out(1, 12, eepromDCLinkUVTripVoltageStr);
                           Lcd_out(2, 1, "Change to: ___ V");
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 12, number1Str);
                           enteredFirstTime = 1;
                           Delay_ms(500);
                       } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setdcLinkUVTripVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                       if(up==0)
                       {
                           upCounter= upCounter+ 1;
                           Delay_ms(1);

                           if(upCounter== 500)
                           {
                               number1=number1+1;
                               if(number1==10)
                               {
                                   number1=0;
                               }
                               upCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                       } // else end
                       if(down==0)
                       {
                           downCounter= downCounter+ 1;
                           Delay_ms(1);
                           if(downCounter== 500)
                           {
                               if(number1==0)
                               {
                                   number1=10;
                               }
                               number1=number1-1;
                               downCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           downCounter= 0;
                       } // else end
                   } // while end

                   while(1)
                   {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "dcLinkUVTripVolt:    V");
                           intToStr(eepromDCLinkUVTripVoltage , eepromDCLinkUVTripVoltageStr, 3);
                           Lcd_out(1, 12, eepromDCLinkUVTripVoltageStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___ V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                      if(back==0)
                      {
                          backCounter= backCounter+ 1;
                          Delay_ms(1);
                          if(backCounter== 500)
                          {
                              uiState = setdcLinkUVTripVoltage;
                              backCounter= 0;
                              backTimer = 1;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          backCounter= 0;
                      } // else end
                      if(enter==0)
                      {
                          enterCounter= enterCounter+ 1;
                          Delay_ms(1);
                          if(enterCounter== 500)
                          {
                              enterCounter= 0;
                              enteredFirstTime = 0;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          enterCounter= 0;
                      } // else end
                      if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter == 500)
                          {
                              number2=number2+1;
                              if(number2==10)
                              {
                                 number2=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter == 500)
                          {
                              if(number2==0)
                              {
                                 number2=10;
                              }
                              number2=number2-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                   } //  while end
                   while(1)                          //
                   {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);
                             Lcd_out(1, 1, "dcLinkUVTripVolt:    V");
                             intToStr(eepromDCLinkUVTripVoltage , eepromDCLinkUVTripVoltageStr, 3);
                             Lcd_out(1, 12, eepromDCLinkUVTripVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             intToStr(number3, number3Str, 1);
                             Lcd_out(2, 14, number3Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setdcLinkUVTripVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                number3=number3+1;
                                if(number3 == 10)
                                {
                                   number3=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(number3 == 0)
                                {
                                   number3 = 10;
                                }
                                number3=number3-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               dcLinkUVTripVoltageInt=(number1*100)+(number2*10)+number3;
                               if(dcLinkUVTripVoltageInt > 010 && dcLinkUVTripVoltageInt < 500)
                               {
                                   intToStr(dcLinkUVTripVoltageInt, dcLinkUVTripVoltageIntStr, 3);
                                   H2 = (dcLinkUVTripVoltageInt>>8);
                                   H1 = (dcLinkUVTripVoltageInt & 0x00FF);
                                   EEPROM_WriteByte(ratedVoltageLocation+14, H2);
                                   EEPROM_WriteByte(ratedVoltageLocation+15, H1);
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "dcLinkUVTripVoltChanged");
                                   Lcd_out(2, 3, "Successfully");
                                   Delay_ms(2000);
                                   uiState = setdcLinkUpperStartVoltage;
                                   dcLinkUVTripVoltage = dcLinkUVTripVoltageInt;
                               }
                               else
                               {
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "Out of Range");
                                   Delay_ms(1500);
                                   uiState = dcLinkUVTripVoltageSetting;
                               }
                               number1 =0;
                               number2 =0;
                               number3 =0;
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                   } //  while end
                   break;
                }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                case setdcLinkUpperStartVoltage:
                {
                    Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                    Lcd_out(1, 1, "Set dcLinkUV");
                    Lcd_out(2, 4, "Trip Voltage?");
                    Delay_ms(500);
                    upCounter = 0;
                    downCounter = 0;
                    enterCounter = 0;
                    backCounter = 0;
                    while(1)
                    {
                       if(back==0)
                       {
                           Delay_ms(1);
                           backCounter= backCounter+ 1;
                           if(backCounter==500)
                           {
                               uiState = restrictedAccess;
                               backCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(enter==0)
                       {
                           Delay_ms(1);
                           enterCounter= enterCounter+ 1;
                           if(enterCounter==500)
                           {
                               uiState = dcLinkUpperStartVoltageSetting;
                               enterCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else if(up==0)
                       {
                           Delay_ms(1);
                           upCounter= upCounter+ 1;
                           if(upCounter==500)
                           {
                               uiState = setdcLinkUVTripVoltage;
                               upCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(down==0)
                       {
                           Delay_ms(1);
                           downCounter= downCounter+ 1;
                           if(downCounter==500)
                           {
                               uiState = setloadCurrentSoftwareTripLimit;
                               downCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                           downCounter= 0;
                           enterCounter = 0;
                           backCounter = 0;
                       }
                    }
                    break;
                }

                case dcLinkUpperStartVoltageSetting:
                {
                    I2 = EEPROM_ReadByte(ratedVoltageLocation+16);
                    I1 = EEPROM_ReadByte(ratedVoltageLocation+17);
                    eepromDCLinkUpperStartVoltage= ((Uint16) I1) | ((Uint16) (I2 << 8));

                   while(1)
                   {
                       if(enteredFirstTime == 0)
                       {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "dcLinkUpperStartVolt:  V");
                           intToStr(eepromDCLinkUpperStartVoltage , eepromDCLinkUpperStartVoltageStr, 3);
                           Lcd_out(1, 12, eepromDCLinkUpperStartVoltageStr);
                           Lcd_out(2, 1, "Change to: ___ V");
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 12, number1Str);
                           enteredFirstTime = 1;
                           Delay_ms(500);
                       } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setdcLinkUpperStartVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                       if(up==0)
                       {
                           upCounter= upCounter+ 1;
                           Delay_ms(1);

                           if(upCounter== 500)
                           {
                               number1=number1+1;
                               if(number1==10)
                               {
                                   number1=0;
                               }
                               upCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                       } // else end
                       if(down==0)
                       {
                           downCounter= downCounter+ 1;
                           Delay_ms(1);
                           if(downCounter== 500)
                           {
                               if(number1==0)
                               {
                                   number1=10;
                               }
                               number1=number1-1;
                               downCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           downCounter= 0;
                       } // else end
                   } // while end

                   while(1)
                   {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "dcLinkUpperStartVolt:    V");
                           intToStr(eepromDCLinkUpperStartVoltage , eepromDCLinkUpperStartVoltageStr, 3);
                           Lcd_out(1, 12, eepromDCLinkUpperStartVoltageStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___ V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                      if(back==0)
                      {
                          backCounter= backCounter+ 1;
                          Delay_ms(1);
                          if(backCounter== 500)
                          {
                              uiState = setdcLinkUpperStartVoltage;
                              backCounter= 0;
                              backTimer = 1;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          backCounter= 0;
                      } // else end
                      if(enter==0)
                      {
                          enterCounter= enterCounter+ 1;
                          Delay_ms(1);
                          if(enterCounter== 500)
                          {
                              enterCounter= 0;
                              enteredFirstTime = 0;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          enterCounter= 0;
                      } // else end
                      if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter == 500)
                          {
                              number2=number2+1;
                              if(number2==10)
                              {
                                 number2=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter == 500)
                          {
                              if(number2==0)
                              {
                                 number2=10;
                              }
                              number2=number2-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                   } //  while end
                   while(1)                          //
                   {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);
                             Lcd_out(1, 1, "dcLinkUpperStartVolt:    V");
                             intToStr(eepromDCLinkUpperStartVoltage , eepromDCLinkUpperStartVoltageStr, 3);
                             Lcd_out(1, 12, eepromDCLinkUpperStartVoltageStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             intToStr(number3, number3Str, 1);
                             Lcd_out(2, 14, number3Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setdcLinkUpperStartVoltage;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                number3=number3+1;
                                if(number3 == 10)
                                {
                                   number3=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(number3 == 0)
                                {
                                   number3 = 10;
                                }
                                number3=number3-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               dcLinkUpperStartVoltageInt=(number1*100)+(number2*10)+number3;
                               if(dcLinkUpperStartVoltageInt > 010 && dcLinkUpperStartVoltageInt < 800)
                               {
                                   intToStr(dcLinkUpperStartVoltageInt, dcLinkUpperStartVoltageIntStr, 3);
                                   I2 = (dcLinkUpperStartVoltageInt>>8);
                                   I1 = (dcLinkUpperStartVoltageInt & 0x00FF);
                                   EEPROM_WriteByte(ratedVoltageLocation+16, I2);
                                   EEPROM_WriteByte(ratedVoltageLocation+17, I1);
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "dcLinkUpperStartVoltChanged");
                                   Lcd_out(2, 3, "Successfully");
                                   Delay_ms(2000);
                                   uiState = setloadCurrentSoftwareTripLimit;
                                   dcLinkUpperStartVoltage = dcLinkUpperStartVoltageInt;
                               }
                               else
                               {
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "Out of Range");
                                   Delay_ms(1500);
                                   uiState = dcLinkUpperStartVoltageSetting;
                               }
                               number1 =0;
                               number2 =0;
                               number3 =0;
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                   } //  while end
                   break;
                }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                case setloadCurrentSoftwareTripLimit:
                {
                    Lcd_Cmd(_LCD_CLEAR);                       // Clear display
                    Lcd_out(1, 1, "Set loadCurrSoft");
                    Lcd_out(2, 4, "Trip Voltage?");
                    Delay_ms(500);
                    upCounter = 0;
                    downCounter = 0;
                    enterCounter = 0;
                    backCounter = 0;
                    while(1)
                    {
                       if(back==0)
                       {
                           Delay_ms(1);
                           backCounter= backCounter+ 1;
                           if(backCounter==500)
                           {
                               uiState = restrictedAccess;
                               backCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(enter==0)
                       {
                           Delay_ms(1);
                           enterCounter= enterCounter+ 1;
                           if(enterCounter==500)
                           {
                               uiState = loadCurrentSoftwareTripLimitSetting;
                               enterCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else if(up==0)
                       {
                           Delay_ms(1);
                           upCounter= upCounter+ 1;
                           if(upCounter==500)
                           {
                               uiState = setdcLinkUpperStartVoltage;
                               upCounter= 0;
                               break;
                           } // if end

                       } // if end
                       else if(down==0)
                       {
                           Delay_ms(1);
                           downCounter= downCounter+ 1;
                           if(downCounter==500)
                           {
                               uiState = setloadCurrentHardwareTripLimit;
                               downCounter= 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                           downCounter= 0;
                           enterCounter = 0;
                           backCounter = 0;
                       }
                    }
                    break;
                }

               case loadCurrentSoftwareTripLimitSetting:
                {
                    J2 = EEPROM_ReadByte(ratedVoltageLocation+18);
                    J1 = EEPROM_ReadByte(ratedVoltageLocation+19);
                    eepromLoadCurrentSoftwareTripLimit= ((Uint16) J1) | ((Uint16) (J2 << 8));

                   while(1)
                   {
                       if(enteredFirstTime == 0)
                       {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "loadCurrSoftTripLim:  V");
                           intToStr(eepromLoadCurrentSoftwareTripLimit , eepromLoadCurrentSoftwareTripLimitStr, 3);
                           Lcd_out(1, 12, eepromLoadCurrentSoftwareTripLimitStr);
                           Lcd_out(2, 1, "Change to: ___ V");
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 12, number1Str);
                           enteredFirstTime = 1;
                           Delay_ms(500);
                       } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setloadCurrentSoftwareTripLimit;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                       if(up==0)
                       {
                           upCounter= upCounter+ 1;
                           Delay_ms(1);

                           if(upCounter== 500)
                           {
                               number1=number1+1;
                               if(number1==10)
                               {
                                   number1=0;
                               }
                               upCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           upCounter= 0;
                       } // else end
                       if(down==0)
                       {
                           downCounter= downCounter+ 1;
                           Delay_ms(1);
                           if(downCounter== 500)
                           {
                               if(number1==0)
                               {
                                   number1=10;
                               }
                               number1=number1-1;
                               downCounter= 0;
                               enteredFirstTime = 0;
                           } // if end
                       } // if end
                       else
                       {
                           downCounter= 0;
                       } // else end
                   } // while end

                   while(1)
                   {
                     if(enteredFirstTime == 0)
                     {
                           Lcd_Cmd(_LCD_CLEAR);                               // Clear display
                           Lcd_out(1, 1, "loadCurrSoftTripLim:    V");
                           intToStr(eepromLoadCurrentSoftwareTripLimit , eepromLoadCurrentSoftwareTripLimitStr, 3);
                           Lcd_out(1, 12, eepromLoadCurrentSoftwareTripLimitStr);
                           intToStr(number1, number1Str, 1);
                           Lcd_out(2, 1, "Change to: ___ V");
                           Lcd_out(2, 12, number1Str);
                           intToStr(number2, number2Str, 1);
                           Lcd_out(2, 13, number2Str);
                           Delay_ms(500);
                           enteredFirstTime = 1;
                      } // if end
                      if(back==0)
                      {
                          backCounter= backCounter+ 1;
                          Delay_ms(1);
                          if(backCounter== 500)
                          {
                              uiState = setloadCurrentSoftwareTripLimit;
                              backCounter= 0;
                              backTimer = 1;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          backCounter= 0;
                      } // else end
                      if(enter==0)
                      {
                          enterCounter= enterCounter+ 1;
                          Delay_ms(1);
                          if(enterCounter== 500)
                          {
                              enterCounter= 0;
                              enteredFirstTime = 0;
                              break;
                          } // if end
                      } // if end
                      else
                      {
                          enterCounter= 0;
                      } // else end
                      if(up==0)
                      {
                          upCounter= upCounter+ 1;
                          Delay_ms(1);
                          if(upCounter == 500)
                          {
                              number2=number2+1;
                              if(number2==10)
                              {
                                 number2=0;
                              }
                              upCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          upCounter= 0;
                      } // else end
                      if(down==0)
                      {
                          downCounter= downCounter+ 1;
                          Delay_ms(1);
                          if(downCounter == 500)
                          {
                              if(number2==0)
                              {
                                 number2=10;
                              }
                              number2=number2-1;
                              downCounter= 0;
                              enteredFirstTime = 0;
                          } // if end
                      } // if end
                      else
                      {
                          downCounter= 0;
                      } // else end
                   } //  while end
                   while(1)                          //
                   {
                       if(enteredFirstTime == 0)
                       {
                             Lcd_Cmd(_LCD_CLEAR);
                             Lcd_out(1, 1, "loadCurrSoftTripLim:    V");
                             intToStr(eepromLoadCurrentSoftwareTripLimit , eepromLoadCurrentSoftwareTripLimitStr, 3);
                             Lcd_out(1, 12, eepromLoadCurrentSoftwareTripLimitStr);
                             intToStr(number1, number1Str, 1);
                             Lcd_out(2, 1, "Change to: ___ V");
                             Lcd_out(2, 12, number1Str);
                             intToStr(number2, number2Str, 1);
                             Lcd_out(2, 13, number2Str);
                             intToStr(number3, number3Str, 1);
                             Lcd_out(2, 14, number3Str);
                             Delay_ms(500);
                             enteredFirstTime = 1;
                        } // if end
                       if(back==0)
                       {
                           backCounter= backCounter+ 1;
                           Delay_ms(1);
                           if(backCounter== 500)
                           {
                               uiState = setloadCurrentSoftwareTripLimit;
                               backCounter= 0;
                               backTimer = 1;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           backCounter= 0;
                       } // else end
                       if(up==0)
                        {
                            upCounter= upCounter+ 1;
                            Delay_ms(1);
                            if(upCounter== 500)
                            {
                                number3=number3+1;
                                if(number3 == 10)
                                {
                                   number3=0;
                                }
                                upCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            upCounter= 0;
                        } // else end
                        if(down==0)
                        {
                            downCounter= downCounter+ 1;
                            Delay_ms(1);
                            if(downCounter== 500)
                            {
                                if(number3 == 0)
                                {
                                   number3 = 10;
                                }
                                number3=number3-1;
                                downCounter= 0;
                                enteredFirstTime = 0;
                            } // if end
                        } // if end
                        else
                        {
                            downCounter= 0;
                        } // else end
                       if(enter==0)
                       {
                           enterCounter= enterCounter+ 1;
                           Delay_ms(1);
                           if(enterCounter== 500)
                           {
                               loadCurrentSoftwareTripLimitInt=(number1*100)+(number2*10)+number3;
                               if(loadCurrentSoftwareTripLimitInt > 010 && loadCurrentSoftwareTripLimitInt < 100)
                               {
                                   intToStr(loadCurrentSoftwareTripLimitInt, loadCurrentSoftwareTripLimitIntStr, 3);
                                   J2 = (loadCurrentSoftwareTripLimitInt>>8);
                                   J1 = (loadCurrentSoftwareTripLimitInt & 0x00FF);
                                   EEPROM_WriteByte(ratedVoltageLocation+16, J2);
                                   EEPROM_WriteByte(ratedVoltageLocation+17, J1);
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "loadCurrentSoftwareTripLimitChanged");
                                   Lcd_out(2, 3, "Successfully");
                                   Delay_ms(2000);
                                   uiState = setloadCurrentHardwareTripLimit;
                                   loadCurrentSoftwareTripLimit = loadCurrentSoftwareTripLimitInt;
                               }
                               else
                               {
                                   Lcd_Cmd(_LCD_CLEAR);
                                   Lcd_out(1, 3, "Out of Range");
                                   Delay_ms(1500);
                                   uiState = loadCurrentSoftwareTripLimitSetting;
                               }
                               number1 =0;
                               number2 =0;
                               number3 =0;
                               enterCounter= 0;
                               enteredFirstTime = 0;
                               break;
                           } // if end
                       } // if end
                       else
                       {
                           enterCounter= 0;
                       } // else end

                   } //  while end
                   break;
                }

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                default:
                      break;
              }
           } // while 1 end
} //end of main()




  //  *************************** end of file ***************************************************** //
