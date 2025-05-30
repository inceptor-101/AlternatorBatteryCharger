#include <IMD_F280033.h>                        // Main include file
#include <string.h>
#include <stdio.h>
#include <math.h>


void faultDetect (void)
{
    FaultDetected = 1;
    Delay_ms(500);
    FaultCodeFlow = 1;
/*
    if(faultCode<15)
    {
        // get the fault counter from 0x0002, increment by 1 and store back at 0x0002
        Addr_2_B=EEPROM_ReadByte(faultCounterLocation);
        Addr_1_B=EEPROM_ReadByte((faultCounterLocation+1));
        faultCounter = ((Uint16) Addr_1_B) | ((Uint16) (Addr_2_B << 8));
        faultCounter=faultCounter+1;
        if(faultCounter>=50)
        {
           faultCounter=50;
        }

        counter= faultCounter;
        C2 = (char)(counter>>8);
        counter= faultCounter;
        C1 = (char) (counter & 0x00FF);

        EEPROM_WriteByte(faultCounterLocation, C2);
        EEPROM_WriteByte(faultCounterLocation+1, C1);
    }

    FaultCodeFlow = 2;
*/
    // ########################Input overvolts#############################
    if(faultCode==1)
    {
        Lcd_Cmd(_LCD_CLEAR);                    // Clear display
        Lcd_out(1, 1, "Grid UnderVolts");
        Delay_ms(1000);

        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=gridundervolts[i];
        }

    } // fault 1 end

    else if(faultCode == 2)
    {
        Lcd_Cmd(_LCD_CLEAR);                     // Clear display
        Lcd_out(1, 1, "Grid OverVolts");
        Delay_ms(1000);
        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=gridovervolts[i];
        }
    } // fault 2 end

    else if(faultCode == 3)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "  Grid Under");
         Lcd_out(2, 1, "  Frequency");
         Delay_ms(1000);
         // prepare data to be written to EEPROM
         for(i=0;i<16;i++)
         {
             faultChunk[i]=underfrequency[i];
         }

     } // fault 3 end

     else if(faultCode == 4)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "  Grid Over");
         Lcd_out(2, 1, "  Frequency");
         Delay_ms(1000);
         // prepare data to be written to EEPROM
         for(i=0;i<16;i++)
         {
             faultChunk[i]=overfrequency[i];
         }

     } // fault 4  end
     if(faultCode == 5)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Output OverVolts");

         // prepare data to be written to EEPROM
         for(i=0;i<16;i++)
         {
             faultChunk[i]=battovervolts[i];
         }

     } // fault 5 end

     else if(faultCode == 6)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "DC Link");
         Lcd_out(2, 1, "OverVoltage");

         // prepare data to be written to EEPROM
         for(i=0;i<16;i++)
         {
             faultChunk[i]=dclinkovervolts[i];
         }

     } // fault 6 end


     else if(faultCode==8)
    {
        Lcd_Cmd(_LCD_CLEAR);                    // Clear display
        Lcd_out(1, 1, "Batt OverCurrent");

        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=battovercurrent[i];
        }

    } // fault 8 end


     else if(faultCode==9)
    {
        FaultCodeFlow = 3;
        Lcd_Cmd(_LCD_CLEAR);                    // Clear display
        Lcd_out(1, 1, "AC OverCurrent");

        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=gridovercurrent[i];
        }

        FaultCodeFlow = 4;

    } // fault 9 end

    else if(faultCode == 10)
    {
        Lcd_Cmd(_LCD_CLEAR);                     // Clear display
        Lcd_out(1, 1, "Hardware Trip");

        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=hardwaretrip[i];
        }

    } // fault 10 end

    else if(faultCode == 11)
    {
        Lcd_Cmd(_LCD_CLEAR);                     // Clear display
        Lcd_out(1, 1, "Transformer");
        Lcd_out(2, 1, "OverCurrent");

        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=TrfOverCurrent[i];
        }

    } // fault 11 end

    else if(faultCode == 12)
    {
       Lcd_Cmd(_LCD_CLEAR);                     // Clear display
       Lcd_out(1, 1, "Softcharge Fail");
       Delay_ms(1000);
       // prepare data to be written to EEPROM
       for(i=0;i<16;i++)
       {
           faultChunk[i]=softChargeFail[i];
       }

    } // fault 12 end

    else if(faultCode == 13)
    {
        Lcd_Cmd(_LCD_CLEAR);                     // Clear display
        Lcd_out(1, 1, "HV Link Low");

        // prepare data to be written to EEPROM
        for(i=0;i<16;i++)
        {
            faultChunk[i]=hvlinklow[i];
        }

    } // fault 13 end

    else if(faultCode == 14)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Thermal Fault");

         // prepare data to be written to EEPROM
         for(i=0;i<16;i++)
         {
             faultChunk[i]=thermalfault[i];
         }

     } // fault 14 end

    else if(faultCode == 15)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "BMS Turn");
         Lcd_out(2, 1, "OFF");
         Delay_ms(1000);

     } // fault 15 end

    else if(faultCode == 20)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Output Current");
         Lcd_out(2, 1, "high side");
         Delay_ms(1000);

     } // fault 20 end

    else if(faultCode == 21)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Output Current");
         Lcd_out(2, 1, "low side");
         Delay_ms(1000);

     } // fault 21 end

    else if(faultCode == 22)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Trf Current");
         Lcd_out(2, 1, "High Side");
         Delay_ms(1000);

     } // fault 22 end

    else if(faultCode == 23)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Trf Current");
         Lcd_out(2, 1, "low side");
         Delay_ms(1000);

     } // fault 23 end

    else if(faultCode == 24)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Grid Current");
         Lcd_out(2, 1, "high side");
         Delay_ms(1000);

     } // fault 22 end

    else if(faultCode == 25)
     {
         Lcd_Cmd(_LCD_CLEAR);                     // Clear display
         Lcd_out(1, 1, "Grid Current");
         Lcd_out(2, 1, "low side");
         Delay_ms(1000);

     } // fault 23 end
    else
    {   Lcd_Cmd(_LCD_CLEAR);                     // Clear display
        Lcd_out(1, 1, "Spurious");
        Lcd_out(2, 1, "Fault");
        Delay_ms(1000);

    }
    // writing faults to memory
/*
    if(faultCode<15)
    {
        secondsBCD = RTC_ReadByte(0x0000);
        minutesBCD = RTC_ReadByte(0x0001);
        hoursBCD   = RTC_ReadByte(0x0002);
        dateBCD    = RTC_ReadByte(0x0004);
        monthBCD   = RTC_ReadByte(0x0005);
        yearBCD    = RTC_ReadByte(0x0006);

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

        // time
        faultChunk[17] = hoursString[0];
        faultChunk[18] = hoursString[1];
        faultChunk[20] = minutesString[0];
        faultChunk[21] = minutesString[1];
        faultChunk[23] = secondsString[0];
        faultChunk[24] = secondsString[1];
        // date
        faultChunk[26] = dateString[0];
        faultChunk[27] = dateString[1];
        faultChunk[29] = monthString[0];
        faultChunk[30] = monthString[1];
        faultChunk[32] = yearString[0];
        faultChunk[33] = yearString[1];

        // read the fault pointer from 0x0000 location on EEPROM
         Addr_2_B=EEPROM_ReadByte(addressStoreLocation);
         Addr_1_B=EEPROM_ReadByte((addressStoreLocation+1));
         faultStorageAddress = ((Uint16) Addr_1_B) | ((Uint16)(Addr_2_B << 8));

        // write to EEPROM memeory
         fsa=faultStorageAddress;
         for(i=0;i<34;i++)
         {
             EEPROM_WriteByte(fsa+i, faultChunk[i]);
         }

         // increment the fault address pointer by 34
         faultStorageAddress=faultStorageAddress+34;

         // check if fault pointer has covered 50 faults
         // if so then reset fault pointer to faultDataStart
         if(faultStorageAddress>=faultDataEnd)
         {
             faultStorageAddress=faultDataStart;
         }

         // *********store current fault address pointer at "0x0000"**********
         address= faultStorageAddress;
         A2B = (char)(address>>8);
         address= faultStorageAddress;
         A1B = (char) (address & 0x00FF);
         EEPROM_WriteByte(addressStoreLocation, A2B);
         EEPROM_WriteByte(addressStoreLocation+1, A1B);


    }
    FaultCodeFlow = 6;
*/
}

