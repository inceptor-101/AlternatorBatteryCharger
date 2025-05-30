#include <AlternatorBatteryChargerF280033.h>         // Main include file
#include "math.h"

void ledCtrlFault(void){

    if (faultype.overcurrent == 1){
        Lcd_Cmd(0x01);                      //Clear Screen
//        Delay_ms(2);
        Lcd_out(1, 1, "Over_Current");
        faultype.overcurrent = 0;           //Reinitialising the fault variable
        return;
    }
    if (faultype.undervoltage == 1){
        gridPresent_led_turnoff;            //Turing grid off
        Lcd_Cmd(0x01);                      //Clear Screen
//        Delay_ms(2);
        Lcd_out(1, 1, "Under_Voltage");
        faultype.undervoltage = 0;          //Reinitialising the fault variable
        return;
    }
    if (faultype.overvoltage == 1){
        gridPresent_led_turnoff;            //Turning grid off
        Lcd_Cmd(0x01);                      //Clear Screen
//        Delay_ms(2);
        Lcd_out(1, 1, "Over_Voltage");
        faultype.overvoltage = 0;           //Reinitialising the fault variable
        return;
    }
    if (faultype.overoutvoltage == 1){
        Lcd_Cmd(0x01);                      //Clear Screen
//        Delay_ms(2);
        Lcd_out(1, 1, "Over_OutputVolt");
        faultype.overoutvoltage = 0;        //Reinitialising the fault variable
        return;
    }
}
