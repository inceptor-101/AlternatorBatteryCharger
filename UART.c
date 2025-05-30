#include <AlternatorBatteryChargerF280033.h>                        // Main include file

void SCIAWrite(char *string)
{
    while(*string)
    {
        while (SciaRegs.SCICTL2.bit.TXEMPTY == 0);
        SciaRegs.SCITXBUF.bit.TXDT = *string++ ;
    }
}

//void addInteger(float val, char* showData){
//    Uint16 integer_part_highvolt = (Uint16)(val);
//
//    showData[2]=(integer_part_highvolt%10)+'0';
//    integer_part_highvolt /= 10;
//
//    showData[1]=(integer_part_highvolt%10)+'0';
//    integer_part_highvolt /= 10;
//
//    showData[0]=(integer_part_highvolt%10)+'0';
//
//    showData[3]='.';
//
//    Uint16 frac_part_highvolt = (Uint16)(val*100);
//
//    showData[5]=frac_part_highvolt%10+'0';
//    frac_part_highvolt /= 10;
//
//    showData[4]=frac_part_highvolt%10+'0';
//    showData[6]='\0';
//}

char* addInteger(float val, char* showData){
    if (val < 0.0){
        *(showData++) = '-';
        val = val*(-1.0);
    }
    else{
        *(showData++) = '+';
    }
//    Integer part
    Uint16 intPart = (Uint16)val;
    val = val - intPart;
    Uint16 fracPart = (Uint16)((val * 100.0) + 0.5f);   //Rounding off to the nearest fractional value

    *(showData++) = (intPart/100)%10 + '0';
    *(showData++) = (intPart/10)%10 + '0';
    *(showData++) = (intPart)%10 + '0';
    *(showData++) = '.';
    *(showData++) = (fracPart/10)%10 + '0';
    *(showData++) = (fracPart)%10 + '0';

//    *(showData++) = '\0';
    return showData;
}

//void publishData(char* txData){
//    *(txData++)='P';
//    *(txData++)='a';
//    *(txData++)='r';
//    *(txData++)='a';
//    *(txData++)='m';
//    *(txData++)=':';
//    txData = addInteger(actualvalues.highvolt, txData);
//    txData = addInteger(actualvalues.currsens, txData);
//    txData = addInteger(actualvalues.outputvolt, txData);
//    txData = addInteger(actualvalues.heatsinkvoltsens, txData);
//    txData = addInteger(actualvalues.inductorcoppvoltsens, txData);
//    txData = addInteger(actualvalues.inductorcorevoltsens, txData);
//    (txData--);
//    *(txData++)='\r';
//    *(txData++)='\n';
//    *(txData)='\0';
//}

void SCIA_init()
{
    SciaRegs.SCICTL1.bit.SWRESET = 0;             //Reset the sci module
//    Configuration for the data frame
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;          //Loop-back is disabled
    SciaRegs.SCICCR.bit.PARITYENA = 0;          //Parity is disabled
    SciaRegs.SCICCR.bit.PARITY = 0;             //No parity
    SciaRegs.SCICCR.bit.SCICHAR = 7;            //Data length is of eight bits
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0x0;    //We are preferring the idle mode
    SciaRegs.SCICCR.bit.STOPBITS = 0x0;         //Only one stop bit will be used

//    Configuration for the receiving and transmitting the objects
    SciaRegs.SCICTL1.bit.TXENA  = 1;              //Now the sci module can send the information
    SciaRegs.SCICTL1.bit.RXENA  = 1;              //The module can now recive the data from the slave
    SciaRegs.SCICTL1.bit.RXERRINTENA = 0;         //Error interrupt is disabled to make sure since it has not been written till now :)

    // SYSCLOCKOUT = 100MHz; LSPCLK = 1/4 = 25.0 MHz
    // BRR = (LSPCLK / (9600 x 8)) -1
    // BRR = 390  gives 9605 Baud

    SciaRegs.SCIHBAUD.bit.BAUD   = 390 >> 8;        // Highbyte
    SciaRegs.SCILBAUD.bit.BAUD   = 390 & 0x00FF;    // Lowbyte
    SciaRegs.SCIFFTX.bit.SCIRST  = 0;               // bit 15 = 1 : relinquish from Reset
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 0;           // pointer is set to 0
    SciaRegs.SCIFFTX.bit.SCIFFENA = 0;              // bit 14 = 1 : Enable FIFO
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 0;            // bit 6 = 0  : TXFFINT-Flag not cleared
    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;              // bit 5 = 0  : disable TX FIFO match
    SciaRegs.SCIFFTX.bit.TXFFIL = 0;                // bit 4-0    :  TX-ISR, if TX FIFO is 0(empty)
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;           // renabled the pointer
    SciaRegs.SCIFFTX.bit.SCIRST  = 1;               // enabling the fifo for the transmission of the data

//    No delay between the successive fifo updates
    SciaRegs.SCIFFCT.all = 0x0000;  // Set FIFO transfer delay to 0

//    For the Rx buffer configuration
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;           // resetting the pointer of the fifo
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;            // reset the last generated interrupt flag on the receive data
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;            // the last generated flag is to be cleared
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;              // interrupt enable on the rx side on the cause
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;               // generate interrupt whenever the data in the fifo crosses 16 bytes
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;           // enabling the pointer for the updation

//    Receive side interrupt
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
//    The interrupts for the receiving and transmitting the uart data
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;  // interrupt enable
//    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;  // interrupt for the transmission is disabled
    IER |= 0x0100;                      // interrupt enable
    SciaRegs.SCICTL1.bit.SWRESET = 1;
}
