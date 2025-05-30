/**********************************************************************
* Code : Solar Inverter for CPU2 - Data Logging Functions
* Devices: TMS320F28379D
* Author: Rashi Chauhan
**********************************************************************/
#include <IMD_F280033.h>                        // Main include file
#include <string.h>
#include <stdio.h>
#include <math.h>

// constants for logging energy data to memory
#define yearMemoryStart 0x5408
#define monthMemoryStart 0x4F57
#define dayMemoryStart 0x0800

// Data Logging every minute
void DataLogging(void)
{
    // read energy(2 chars) from memory and write it back to memory
    // after adding energy accumulated in one minute
    // for present day
    Addr_2_B=EEPROM_ReadByte(0x0005);
    Addr_1_B=EEPROM_ReadByte(0x0006);
    daypointer = ((Uint16) Addr_1_B) | ((Uint16) (Addr_2_B << 8));

    Addr_2_B=EEPROM_ReadByte(daypointer);
    Addr_1_B=EEPROM_ReadByte(daypointer+1);
    energyAccumulated = ((Uint16) Addr_1_B) | ((Uint16) (Addr_2_B << 8));
    energyAccumulated = energyAccumulated + energyint;

    // check Daily energy write to memory code: start
    if((energyAccumulated < energyAccumulatedPrev) && (timeToSwitchOverDay == 0))
    {
        energyAccumulated = energyAccumulatedPrev;
    }
    // check Daily energy write to memory code: end

    energyWrite= energyAccumulated;
    E2B = (char)(energyWrite>>8);
    energyWrite= energyAccumulated;
    E1B = (char) (energyWrite & 0x00FF);
    EEPROM_WriteByte(daypointer, E2B);
    EEPROM_WriteByte(daypointer+1, E1B);

    // for present month
    Addr_2_B=EEPROM_ReadByte(0x0007);
    Addr_1_B=EEPROM_ReadByte(0x0008);
    monthpointer = ((Uint16) Addr_1_B) | ((Uint16) (Addr_2_B << 8));

    Addr_4_B=EEPROM_ReadByte(monthpointer);
    Addr_3_B=EEPROM_ReadByte(monthpointer+1);
    Addr_2_B=EEPROM_ReadByte(monthpointer+2);
    Addr_1_B=EEPROM_ReadByte(monthpointer+3);
    energyAccumulated2 = (((Uint32) Addr_4_B) << 24) | (((Uint32) Addr_3_B) << 16) | (((Uint32) Addr_2_B) << 8) | ((Uint32) Addr_1_B);
    energyAccumulated2 = energyAccumulated2 + energyint;

    energyWrite2= energyAccumulated2;
    E4B = (char)(energyWrite2>>24);
    energyWrite2= energyAccumulated2;
    E3B = (char) ((energyWrite2>>16) & 0x000000FF);
    energyWrite2= energyAccumulated2;
    E2B = (char)((energyWrite2>>8) & 0x000000FF);
    energyWrite2= energyAccumulated2;
    E1B = ( char) (energyWrite2 & 0x000000FF);

    EEPROM_WriteByte(monthpointer, E4B);
    EEPROM_WriteByte(monthpointer+1, E3B);
    EEPROM_WriteByte(monthpointer+2, E2B);
    EEPROM_WriteByte(monthpointer+3, E1B);

    // for present year
    Addr_2_B=EEPROM_ReadByte(0x0009);
    Addr_1_B=EEPROM_ReadByte(0x000A);
    yearpointer = ((Uint16) Addr_1_B) | ((Uint16) (Addr_2_B << 8));

    Addr_4_B=EEPROM_ReadByte(yearpointer);
    Addr_3_B=EEPROM_ReadByte(yearpointer+1);
    Addr_2_B=EEPROM_ReadByte(yearpointer+2);
    Addr_1_B=EEPROM_ReadByte(yearpointer+3);
    energyAccumulated2 = (((Uint32) Addr_4_B) << 24) | (((Uint32) Addr_3_B) << 16) | (((Uint32) Addr_2_B) << 8) | ((Uint32) Addr_1_B);
    energyAccumulated2 = energyAccumulated2 + energyint;

    energyWrite2= energyAccumulated2;
    E4B = (char)(energyWrite2>>24);
    energyWrite2= energyAccumulated2;
    E3B = (char) ((energyWrite2>>16) & 0x000000FF);
    energyWrite2= energyAccumulated2;
    E2B = (char)((energyWrite2>>8) & 0x000000FF);
    energyWrite2= energyAccumulated2;
    E1B = (char) (energyWrite2 & 0x000000FF);

    EEPROM_WriteByte(yearpointer, E4B);
    EEPROM_WriteByte(yearpointer+1, E3B);
    EEPROM_WriteByte(yearpointer+2, E2B);
    EEPROM_WriteByte(yearpointer+3, E1B);

    energy=0;
    energyint=0;
    // check Daily energy write to memory code: start
    energyAccumulatedPrev = energyAccumulated;
    // check Daily energy write to memory code: end
}
// convert year to memory
Uint16 year2memory(Uint16 yy)
{
    Uint16 memy;
    memy = yearMemoryStart+(yy-21)*4;
    return memy;
}  // function end

// convert month to memory
Uint16 month2memory(Uint16 yy, Uint16 mm)
{
    Uint16 memm;
    memm = monthMemoryStart+(yy-21)*48+(mm-1)*4;
    return memm;
}  // function end

// convert day to memory
Uint16 day2memory(Uint16 yy, Uint16 mm, Uint16 dd)
{
    Uint16 mem;
    if(yy<24)
    {
        if(mm==1)
        {
            mem=dayMemoryStart+((yy-21)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=dayMemoryStart+((yy-21)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2021-2023 if end

    else if(yy==24)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*3))+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*3))+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*3))+62+58+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*3))+62+58+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2024 if end
    else if(yy<28)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*4)+2)+((yy-25)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2025-2027 if end
    else if(yy == 28)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*7)+2)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*7)+2)+62+58+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    } // 2028 if end
    else if(yy<32)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*8)+4)+((yy-29)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2029-2031 if end
    else if(yy == 32)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*11)+4)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*11)+4)+62+58+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    } // 2032 if end
    else if(yy<36)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*12)+6)+((yy-33)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2033-2035 if end
    else if(yy == 36)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*15)+6)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*15)+6)+62+58+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    } // 2036 if end
    else if(yy<40)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*16)+8)+((yy-37)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2037-2039 if end
    else if(yy == 40)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*19)+8)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*19)+8)+62+58+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    } // 2040 if end
    else if(yy<44)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*20)+10)+((yy-41)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2041-2043 if end
    else if(yy == 44)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*23)+10)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*23)+10)+62+58+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    } // 2044 if end
    else if(yy<48)
    {
        if(mm==1)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+2*(dd-1);
        }
        else if(mm==2)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+2*(dd-1);
        }
        else if(mm==3)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+2*(dd-1);
        }
        else if(mm==4)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+2*(dd-1);
        }
        else if(mm==5)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+2*(dd-1);
        }
        else if(mm==6)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+2*(dd-1);
        }
        else if(mm==7)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+60+2*(dd-1);
        }
        else if(mm==8)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+60+62+2*(dd-1);
        }
        else if(mm==9)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+60+62+62+2*(dd-1);
        }
        else if(mm==10)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+60+62+62+60+2*(dd-1);
        }
        else if(mm==11)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+60+62+62+60+62+2*(dd-1);
        }
        else if(mm==12)
        {
            mem=(dayMemoryStart+(730*24)+12)+((yy-45)*730)+62+56+62+60+62+60+62+62+60+62+60+2*(dd-1);
        }
    }  // 2045-2047 if end

    return mem;

} // function end



