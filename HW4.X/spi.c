#include<xc.h>           // processor SFR definitions
#include "spi.h"

void initSPI1() {
    RPA4Rbits.RPA4R = 0011; //SDO1 on A4
    RPB7Rbits.RPB7R = 0011; //SS1 on B7
    SDI1Rbits.SDI1R = 0100; //SDI1 on B8
                            //SCK1 on B14
    
    SPI1BUF;
    SPI1BRG = 0x18; // baud rate to 1MHz
    SPI1STATbits.SPIROV = 0;
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.ON = 1;
    
}