#include <xc.h>           // processor SFR definitions
#include "spi.h"

int main() {
__builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here

    __builtin_enable_interrupts();
    initSPI1();
    
  
    
    while(1)
    {  
        LATBbits.LATB7 = 0; //set to HIGH initially, LED is on
        SPI1BUF = 0b111110000000;
        LATBbits.LATB7 = 1;
        SPI1BUF; //read from buf to clear
    }
    
    return 0;
}