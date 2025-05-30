
//###########################################################################
//
// FILE:   f28002x_gpio.c
//
// TITLE:  f28002x GPIO module support functions
//
//###########################################################################
// $TI Release: F28002x Support Library v3.01.00.00 $
// $Release Date: Thu Mar 19 07:50:20 IST 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
////
//#include "driverlib.h"
//#include "device.h"
#include <AlternatorBatteryChargerF280033.h>                        // Main include file
#include "f28003x_device.h"
#include "f28003x_examples.h"
//
// Low-level functions for GPIO configuration
//

//
// InitGpio - Sets all pins to be muxed to GPIO in input mode with pull-ups
// enabled.Also resets CPU control to CPU1 and disables open drain
// and polarity inversion and sets the qualification to synchronous.
// Also unlocks all GPIOs. Only one CPU should call this function.
//
void
InitGpio()
{

// ################################################################################################
// ---------   Unlock the GPxLOCK register bits for all ports --------------
// ################################################################################################

    GpioCtrlRegs.GPACR.all   = 0x00000000;
    GpioCtrlRegs.GPBCR.all   = 0x00000000;
    GpioCtrlRegs.GPHCR.all   = 0x00000000;

// ################################################################################################
// ---------   Disable the register locks for all ports --------------
// ################################################################################################

    EALLOW;     // ??

    GpioCtrlRegs.GPALOCK.all = 0x00000000;
    GpioCtrlRegs.GPBLOCK.all = 0x00000000;
    GpioCtrlRegs.GPHLOCK.all = 0x00000000;

// ################################################################################################
// ---------   Group-A GPIO Pins --------------
// ################################################################################################

    GpioCtrlRegs.GPACTRL.all  = 0x00000000;     // QUALPRD = PLLSYSCLK for all group A GPIO
    GpioCtrlRegs.GPAQSEL1.all = 0x00000000;     // Synchronous qualification for all group A GPIO 0-15
    GpioCtrlRegs.GPAQSEL2.all = 0x00000000;     // Synchronous qualification for all group A GPIO 16-31
    GpioCtrlRegs.GPADIR.all   = 0x00000000;     // All GPIO are inputs
    GpioCtrlRegs.GPAPUD.all   = 0x00000000;     // All pullups enabled
    GpioCtrlRegs.GPAINV.all   = 0x00000000;     // No inputs inverted
    GpioCtrlRegs.GPAODR.all   = 0x00000000;     // All outputs normal mode (no open-drain outputs)

// ################################################################################################
// ---------   Group-B GPIO Pins --------------
// ################################################################################################

    GpioCtrlRegs.GPBCTRL.all  = 0x00000000;     // QUALPRD = PLLSYSCLK for all group B GPIO
    GpioCtrlRegs.GPBQSEL1.all = 0x00000000;     // Synchronous qualification for all group B GPIO 32-47
    GpioCtrlRegs.GPBQSEL2.all = 0x00000000;     // Synchronous qualification for all group B GPIO 48-63
    GpioCtrlRegs.GPBDIR.all   = 0x00000000;     // All group B GPIO are inputs
    GpioCtrlRegs.GPBPUD.all   = 0x00000000;     // All group B pullups enabled
    GpioCtrlRegs.GPBINV.all   = 0x00000000;     // No inputs inverted
    GpioCtrlRegs.GPBODR.all   = 0x00000000;     // All outputs normal mode (no open-drain outputs)

// ################################################################################################
// ---------   Group-H GPIO Pins --------------
// ################################################################################################

    GpioCtrlRegs.GPHCTRL.all  = 0x00000000;     // QUALPRD = PLLSYSCLK for all group H GPIO
    GpioCtrlRegs.GPHQSEL1.all = 0x00000000;     // Synchronous qualification for all group H GPIO 224-239
    GpioCtrlRegs.GPHQSEL2.all = 0x00000000;     // Synchronous qualification for all group H GPIO 240-255
    GpioCtrlRegs.GPHINV.all   = 0x00000000;     // No inputs inverted

    // Pin Out

    // PWM Pins
    /*
     GPIO0- PWM 1A
     GPIO1- PWM 1B
     GPIO2- PWM 2A
     GPIO3- PWM 2B
     GPIO4- PWM 3A
     GPIO5- PWM 3B
     GPIO6- PWM 4A  (Brake Chopper Control)
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

     */

    // I2C Communication
    /*
     GPIO18 - SCL
     GPIO19 - SDA
     */
    // ################################################################################################
    // ---------------------------------  GPIO pins configuration -------------------------------------
    // ################################################################################################

    // Configuring the pins as analog pins for ADCs and DAC
        GpioCtrlRegs.GPHAMSEL.bit.GPIO224 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO226 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO228 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO231 = 1;    // DAC
        GpioCtrlRegs.GPHAMSEL.bit.GPIO232 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO237 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO238 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO239 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO244 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO245 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO248 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO249 = 1;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;       // PWM
        GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;       // PWM
        GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 1;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 2;       // SCIA_TX configuration
        GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;          //Disabling the pull up
        GpioCtrlRegs.GPAQSEL1.bit.GPIO2 = 3;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 2;       // SCIA_RX configuration
        GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 1;
        GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;       // PWM
        GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;       // PWM
        GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 1;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;       // Digital Output (Brake Chopper Control)
        GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;       //GPIO-7: Main Relay
        GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0;

//      -----------------------Configuration of the LEDs-----------------------------------------------------
        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;       //GPIO-8: LED1
        GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;       //GPIO-9: LED2
        GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;       //GPIO-10: LED3
        GpioCtrlRegs.GPAMUX1.bit.GPIO10  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;       //GPIO-11: LED4
        GpioCtrlRegs.GPAMUX1.bit.GPIO11  = 0;

        //--------------------------End of the configuration part----------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;       //D5, LCD
        GpioCtrlRegs.GPAMUX1.bit.GPIO12  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;       //D6, LCD
        GpioCtrlRegs.GPAMUX1.bit.GPIO13  = 0;


        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;       //EN, LCD
        GpioCtrlRegs.GPAMUX2.bit.GPIO16  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0;       //RS, LCD
        GpioCtrlRegs.GPAMUX2.bit.GPIO17  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 1;       //SCL
        GpioCtrlRegs.GPAMUX2.bit.GPIO18  = 2;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 1;       //SDA
        GpioCtrlRegs.GPAMUX2.bit.GPIO19  = 2;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 0;       //SW1
        GpioCtrlRegs.GPAMUX2.bit.GPIO22  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0;       //SW3
        GpioCtrlRegs.GPAMUX2.bit.GPIO23  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = 0;       // Cooling Fan Control
        GpioCtrlRegs.GPAMUX2.bit.GPIO24  = 0;


        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 0;       //D7, LCD
        GpioCtrlRegs.GPAMUX2.bit.GPIO28  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 0;       //GPIO-29 Soft Charge Relay
        GpioCtrlRegs.GPAMUX2.bit.GPIO29  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = 0;       //GPIO-29 Soft Charge Relay
        GpioCtrlRegs.GPBMUX1.bit.GPIO32  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPBGMUX1.bit.GPIO33 = 0;       //D4, LCD
        GpioCtrlRegs.GPBMUX1.bit.GPIO33  = 0;

        //-----------------------------------------------------------------------------------------------------
        //GpioCtrlRegs.GPBGMUX1.bit.GPIO37 = 0;       //Digital Input, ON/OFF control for the VFD
        //GpioCtrlRegs.GPBMUX1.bit.GPIO37  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 0;       //SW4
        GpioCtrlRegs.GPBMUX1.bit.GPIO40  = 0;

        //-----------------------------------------------------------------------------------------------------
        GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 0;       //SW2
        GpioCtrlRegs.GPBMUX1.bit.GPIO41  = 0;




    // ################################################################################################
    // ----------------------------  Initialization of GPIOs as OUTPUT Pins ---------------------------
    // ################################################################################################

        // ******* Making GPIO pins as Output Pins: START *******

        GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;     // Brake Chopper Control
        GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
//        8-11 GPIO pins means for the LEDs
        GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;
//        GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
        GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;

        // ******* Making GPIO pins as Output Pins: END *******

        // ******* Making GPIO pins as Input Pins: START *******(mostly the switches for the LCD)

        GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;
        GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;

        // ******* Making GPIO pins as Input Pins: END *******

        GpioCtrlRegs.GPAQSEL2.bit.GPIO22   = 2;       // qualify GPIO to 6 samples
        GpioCtrlRegs.GPAQSEL2.bit.GPIO23   = 2;       // qualify GPIO to 6 samples
        GpioCtrlRegs.GPBQSEL1.bit.GPIO32   = 2;       // qualify GPIO to 6 samples
        GpioCtrlRegs.GPBQSEL1.bit.GPIO40   = 2;       // qualify GPIO to 6 samples
        GpioCtrlRegs.GPBQSEL1.bit.GPIO41   = 2;       // qualify GPIO to 6 samples

        GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 255;     // maximum delay
        GpioCtrlRegs.GPBCTRL.bit.QUALPRD0 = 255;     // maximum delay
        GpioCtrlRegs.GPBCTRL.bit.QUALPRD1 = 255;     // maximum delay

// ################################################################################################
// --------------------------- Enable the register locks for all ports ----------------------------
// ################################################################################################

    GpioCtrlRegs.GPALOCK.all = 0xFFFFFFFF;
    GpioCtrlRegs.GPBLOCK.all = 0xFFFFFFFF;
    GpioCtrlRegs.GPHLOCK.all = 0xFFFFFFFF;

// ################################################################################################
// --------------------   Lock the GPxLOCK register bits for all ports ----------------------------
// ################################################################################################

    GpioCtrlRegs.GPACR.all   = 0xFFFFFFFF;
    GpioCtrlRegs.GPBCR.all   = 0xFFFFFFFF;
    GpioCtrlRegs.GPHCR.all   = 0xFFFFFFFF;

    EDIS;
}



