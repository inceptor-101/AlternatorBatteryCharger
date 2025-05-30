#include <AlternatorBatteryChargerF280033.h>

//Going for the LED control
void ledCtrl(Uint16 state){
    EALLOW;
    switch(state){
        case turnon1:
            GpioDataRegs.GPASET.bit.GPIO8 = 1;
            break;
        case toggle1:
            GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;
            break;
        case turnoff1:
            GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
            break;
        case turnon2:
            GpioDataRegs.GPASET.bit.GPIO9 = 1;
            break;
        case toggle2:
            GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
            break;
        case turnoff2:
            GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
            break;
        case turnon3:
            GpioDataRegs.GPASET.bit.GPIO10 = 1;
            break;
        case toggle3:
            GpioDataRegs.GPATOGGLE.bit.GPIO10 = 1;
            break;
        case turnoff3:
            GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
            break;
        case turnon4:
            GpioDataRegs.GPASET.bit.GPIO11 = 1;
            break;
        case toggle4:
            GpioDataRegs.GPATOGGLE.bit.GPIO11 = 1;
            break;
        case turnoff4:
            GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
            break;
    }
    EDIS;
}

void dispParamsOnLED(void){
    char* ptr;
    switch (whatToShow){
        case show_state: {
            if (currstate != prevstate){
                Lcd_Cmd(0x01);
                prevstate = currstate;
            }
            switch (currstate) {
                case volt_lim_mode: {
                    Lcd_out(1,1,"Volt_lim_mode");
                    break;
                }
                case one_sec_delay_mode: {
                    Lcd_out(1,1,"1s_delay_mode");
                    break;
                }
                case two_sec_off_avg_mode: {
                    Lcd_out(1,1,"2s_delay_mode");
                    break;
                }
                case activelogic: {
                    Lcd_out(1,1,"Active_mode");
                    break;
                }
                case triplogic: {
                    Lcd_out(1,1,"Triplogic_mode");
                    break;
                }
            }   // End of the state display logic
            break;
        }   // End of logic to display the state of the charger.

        case show_output_volt: {
//            Lcd_Cmd(0x01);      //Cleared the screen
            ptr = addInteger(avgvalues.avgoutvoltage, showData);
            (*ptr++) = 'V';
            (*ptr) = '\0';
            Lcd_out(1,1,"Output Voltage:");
            Lcd_out(2,1, showData);
            break;
        }
        case show_curr: {
//            Lcd_Cmd(0x01);      //Cleared the screen
            ptr = addInteger(avgvalues.avgcurrsens, showData);
            (*ptr++) = 'A';
            (*ptr) = '\0';
            Lcd_out(1,1,"Output Current:");
            Lcd_out(2,1, showData);
            break;
        }
        case show_dc_link_volt: {
//            Lcd_Cmd(0x01);      //Cleared the screen
            ptr = addInteger(avgvalues.avgdclinkvoltage, showData);
            (*ptr++) = 'V';
            (*ptr) = '\0';
            Lcd_out(1,1,"DC_Link Voltage:");
            Lcd_out(2,1, showData);
            break;
        }
        case show_heat_sink_temp: {
//            Lcd_Cmd(0x01);      //Cleared the screen
            ptr = addInteger(avgvalues.avgheatsinktemp, showData);
            (*ptr) = '\0';
            Lcd_out(1,1,"Heat sink temp:");
            Lcd_out(2,1, showData);
            LcdData(1);
            Lcd_out(2,9, "C");
            break;
        }
        case show_ind_copp_temp: {
//            Lcd_Cmd(0x01);      //Cleared the screen
            ptr = addInteger(avgvalues.avgindcopptemp, showData);
            (*ptr) = '\0';
            Lcd_out(1,1,"Ind copp temp:");
            Lcd_out(2,1, showData);
            LcdData(1);
            Lcd_out(2,9, "C");
            break;
        }
        case show_ind_core_temp: {
//            Lcd_Cmd(0x01);      //Cleared the screen
            ptr = addInteger(avgvalues.avgindcoretemp, showData);
            (*ptr) = '\0';
            Lcd_out(1,1,"Ind core temp:");
            Lcd_out(2,1, showData);
            LcdData(1);
            Lcd_out(2,9, "C");
            break;
        }
    }
}
