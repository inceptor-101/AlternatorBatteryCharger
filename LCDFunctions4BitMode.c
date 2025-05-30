
#include <AlternatorBatteryChargerF280033.h>                        // Main include file
#include <string.h>
#include <stdio.h>
#include <math.h>

Uint16 stringLen(char* ipStr)
{
    Uint16 i = 0;
    while (ipStr[i] != '\0')
    {
       i = i + 1;
    }
    return i;
}

///  **Command**           | **Hex Code**     | **Binary Format** | **Effect / Description**
///  ----------------------|------------------|--------------------|--------------------------------------------
///  **Clear Display**     | 0x01             | 00000001           | Clears entire display, sets address counter (AC) to 0. Takes ~1.6 ms
///  **Return Home**       | 0x02             | 00000010           | Sets AC to 0. Cursor returns to home. Takes ~1.6 ms
///  **Entry Mode Set**    | 0x04 - 0x07      | 000001IS           | I=1: Increment AC, I=0: Decrement. S=1: Shift display
///     Example            | 0x06             | 00000110           | Increment AC, no shift (typical)
///  **Display ON/OFF**    | 0x08 - 0x0F      | 00001DCB           | D: Display ON, C: Cursor ON, B: Blink ON
///     Example            | 0x0C             | 00001100           | Display ON, cursor OFF, blink OFF (typical)
///     Example            | 0x0E             | 00001110           | Display ON, cursor ON, blink OFF
///     Example            | 0x0F             | 00001111           | Display ON, cursor ON, blink ON
///  **Cursor/Display Shift** | 0x10 - 0x1F   | 0001SCRL           | S=1: Shift display, R/L=1: Right, 0: Left
///     Example            | 0x18             | 00011000           | Shift entire display left
///     Example            | 0x1C             | 00011100           | Shift entire display right
///  **Function Set**      | 0x20 - 0x3F      | 001DLNFx           | DL=1: 8-bit, DL=0: 4-bit; N=1: 2-line, F=1: 5x10 font
///     Example            | 0x28             | 00101000           | 4-bit, 2-line, 5x8 dots (standard config)
///  **Set CGRAM Address** | 0x40 - 0x7F      | 01xxxxxx           | Sets AC to CGRAM address for defining custom characters
///  **Set DDRAM Address** | 0x80 - 0xFF      | 1xxxxxxx           | Sets AC to DDRAM address (e.g., 0x80 = line 1 start, 0xC0 = line 2 start)

void LcdInit(void)
{
    Delay_ms(15);               /*15ms,16x2 LCD Power on delay*/
    Lcd_Cmd(0x02);              /*send for initialization of LCD
                                  for nibble (4-bit) mode */
    Lcd_Cmd(0x28);              /*use 2 line and
                                  initialize 5*8 matrix in (4-bit mode)*/
    Lcd_Cmd(0x08);              /*it means the display is set to off*/
    Lcd_Cmd(0x01);              /*clear display screen*/
    Delay_ms(3);
    Lcd_Cmd(0x0C);              /*display on cursor off*/
//    Lcd_Cmd(0x0f);              /*increment cursor (shift cursor to right)*/
}

void Lcd_Cmd(unsigned char value)
{
    // Upper Nibble
    if ((value & 0x10) == 0)
      d4OFF = 1;
    else
      d4ON = 1;

    if ((value & 0x20) == 0)
      d5OFF = 1;
    else
      d5ON = 1;

    if ((value & 0x40) == 0)
      d6OFF = 1;
    else
      d6ON = 1;

    if ((value & 0x80) == 0)
      d7OFF = 1;
    else
      d7ON = 1;

    rsPinOFF = 1;  /*Command Register is selected i.e.RS=0*/
    enablePinON = 1;  /*High-to-low pulse on Enable pin to latch data*/
    Delay_ms(1);
    enablePinOFF = 1;
    Delay_ms(1);

    // Lower Nibble
    if ((value & 0x01) == 0)
       d4OFF = 1;
    else
       d4ON = 1;

    if ((value & 0x02) == 0)
      d5OFF = 1;
    else
      d5ON = 1;
    if ((value & 0x04) == 0)
       d6OFF = 1;
    else
       d6ON = 1;
    if ((value & 0x08) == 0)
        d7OFF = 1;
    else
        d7ON = 1;

    enablePinON = 1;  /*High-to-low pulse on Enable pin to latch data*/
    Delay_ms(1);
    enablePinOFF = 1;
    Delay_ms(15);
}
void LcdData(unsigned char value)
{
    // Upper Nibble
    if ((value & 0x10) == 0)
      d4OFF = 1;
    else
      d4ON = 1;

    if ((value & 0x20) == 0)
      d5OFF = 1;
    else
      d5ON = 1;

    if ((value & 0x40) == 0)
      d6OFF = 1;
    else
      d6ON = 1;

    if ((value & 0x80) == 0)
      d7OFF = 1;
    else
      d7ON = 1;

    rsPinON = 1;  /*Data Register is selected*/
    enablePinON = 1;  /*High-to-low pulse on Enable pin to latch data*/
    Delay_ms(1);
    enablePinOFF = 1;
    Delay_ms(1);

    //Lower Nibble
    if ((value & 0x01) == 0)
       d4OFF = 1;
    else
       d4ON = 1;

    if ((value & 0x02) == 0)
      d5OFF = 1;
    else
      d5ON = 1;
    if ((value & 0x04) == 0)
       d6OFF = 1;
    else
       d6ON = 1;
    if ((value & 0x08) == 0)
        d7OFF = 1;
    else
        d7ON = 1;

    enablePinON = 1;  /*High-to-low pulse on Enable pin to latch data*/
    Delay_ms(1);
    enablePinOFF = 1;
    Delay_ms(3);
}
void Lcd_out(Uint16 rowNumber, Uint16 colNumber, char* dataDisplay)
{
    Uint16 j,len;
    unsigned char Display , rowCount;
    len = stringLen(dataDisplay);
    if (rowNumber == 1)
    {
        rowCount = 127 + colNumber;
        Lcd_Cmd(rowCount);
    }
    else
    {
        rowCount = 191 + colNumber;
        Lcd_Cmd(rowCount);
    }

    for (j= 0; j < len ;j++)
    {
       Display = dataDisplay[j];
       LcdData(Display);
    }
}

// Define custom character (emoji) at location 0
void Lcd_CreateCustomChar(Uint16 location, unsigned char* pattern) {
    Uint16 i;
    if (location < 8) {
        Lcd_Cmd(0x40 + (location * 8));  // Set CGRAM address for custom char
        for (i = 0; i < 8; i++) {
            LcdData(pattern[i]);        // Write 8 bytes (5x8 matrix)
        }
    }
}





