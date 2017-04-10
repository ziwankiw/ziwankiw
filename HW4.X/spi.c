#include "spi.h"

void initSPI1() {
    
    SPI1CON = 0;
    SPI1BUF;
    SPI1BRG = 23; // 23  -> baud rate to 1MHz
                  // 239 -> baud 100kHz for debugging
    SPI1STATbits.SPIROV = 0;
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.ON = 1;
    
    RPA4Rbits.RPA4R = 0b0011; //SDO1 on A4
    //RPB7Rbits.RPB7R = 0011; //SS1 on B7
    TRISBbits.TRISB7 = 0; //B7 is SS1 output
        
    SDI1Rbits.SDI1R = 0b0100; //SDI1 on B8
                            //SCK1 on B14
    
    makeSineWave();
    makeTriangleWave();
}

void makeSineWave() {
    int i=0;
    for(i=0; i<1000; ++i)
    {
        sinewave[i]=128+127*sin(PI*i/100);
    }
}

void makeTriangleWave() {
    int i=0;
    for(i=0; i<1000; ++i)
    {
        triwave[i] = ((unsigned char) (1.275*i)) %255;    
    }
}

void setVoltage(unsigned char channel, unsigned char voltage)
{
    unsigned char byte1 = 0;
    unsigned char byte2 = 0;
    
    if(channel == 0){
        byte1 = 0b01110000;
    }
    else {
        byte1 = 0b11110000; 
    }
    
    unsigned char vbyte1 = voltage >> 4;
    byte1 = byte1|vbyte1;
    
    byte2 = voltage<<4;
    
    CS = 0; //CS pin low -> begin data transmission
    SPI1_IO(byte1);
    SPI1_IO(byte2);
    CS = 1; //CS pin high -> end data transmission
    
    
}

char SPI1_IO(unsigned char write)
{
        
        SPI1BUF = write;   
        while(!SPI1STATbits.SPIRBF) {;}        
        
    return SPI1BUF;
}