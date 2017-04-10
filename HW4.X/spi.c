#include "spi.h"

void initSPI1() {
    
    SPI1CONbits.ON = 0;
    SPI1BUF;
    SPI1BRG = 239; // baud rate to 1MHz
    SPI1STATbits.SPIROV = 0;
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 1;
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
        int val = i/100;
        sinewave[i]=128+127*sin(PI*val);
    }
}

void makeTriangleWave() {
    int i=0;
    for(i=0; i<1000; ++i)
    {
        triwave[i] = ((int) (1.275*i)) %255;    
    }
}
