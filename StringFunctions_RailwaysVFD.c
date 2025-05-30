/**********************************************************************
* Code : Solar Inverter for CPU2 - String Functions
* Devices: TMS320F28379D
* Author: Rashi Chauhan
**********************************************************************/
#include <AlternatorBatteryChargerF280033.h>                        // Main include file
#include <string.h>
#include <stdio.h>
#include <math.h>

void reverseString (char* Str, Uint16 len )
{
    Uint16 i=0;
    Uint16 j,temp;
    j = len - 1;
    while (i<j)
       {
           temp = Str[i];
           Str[i] = Str[j];
           Str[j] = temp;
           i++;
           j--;
       }
}

Uint16 intToStr(Uint16 x, char str[], Uint16 d) // d represents minimum number of digits
{
    Uint16 i = 0;
    while (x)
    {
        str[i] = (x%10) + '0';
        x = x/10;
        i = i + 1;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
    {  str[i] = '0';
       i = i + 1;
    }

    reverseString(str, i);
    str[i] = '\0';
    return i;
}

//Uint16 atoi(char *res, Uint16 digits)  // minimum value of digits should be 1
//{
//    Uint16 ind = 0;
//    Uint16 result = 0;
//
//    for(ind=0; ind<digits; ind++)
//    {
//        result = result + (res[ind]-'0')*pow(10, digits - (ind+1));
//    }
//    return result;
//}

Uint16 int32ToStr(Uint32 x, char str[], Uint16 d) // d represents minimum number of digits
{
    Uint16 i = 0;
    while (x)
    {
        str[i] = (x%10) + '0';
        x = x/10;
        i = i + 1;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
    {  str[i] = '0';
       i = i + 1;
    }

    reverseString(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char *res, Uint16 afterpoint)  // minimum value of afterpoint should be 1
{
    // Extract integer part
    Uint16 ipart = (Uint16) n;

    // Extract floating part
    float fpart = n - (float) ipart;

    // convert integer part to string
    Uint16 i = intToStr(ipart, res, 1); // minimum digits before decimal is 1

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * powf(10, afterpoint);

        intToStr((Uint16) fpart, res + i + 1, afterpoint);
    }
}

void f64toa(float n, char *res, Uint16 afterpoint)  // minimum value of afterpoint should be 1
{
    // Extract integer part
    Uint16 ipart = (Uint16) n;

    // Extract floating part
    float fpart = n - (float) ipart;

    // convert integer part to string
    Uint16 i = intToStr(ipart, res, 1); // minimum digits before decimal is 1

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * powf(10, afterpoint);

        int32ToStr((Uint32) fpart, res + i + 1, afterpoint);
    }
}


// Function to convert a float to a string
void floatToString(float num, char* str, int precision) {
    int integerPart = (int)num;
    float fractionalPart = num - integerPart;

    // Convert integer part to string
    int i = 0;
    while (integerPart > 0) {
        str[i++] = '0' + integerPart % 10;
        integerPart /= 10;
    }

    // Reverse the integer part in the string
    int start = 0, end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }

    // Add decimal point
    if (precision > 0) {
        str[i++] = '.';
    }

    // Convert fractional part to string with specified precision
    while (precision > 0) {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;
        str[i++] = '0' + digit;
        fractionalPart -= digit;
        precision--;
    }

    // Null-terminate the string
    str[i] = '\0';
}



