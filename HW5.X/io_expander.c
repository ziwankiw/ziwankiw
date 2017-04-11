#include "io_expander.h"
#include "i2c_master_noint.h"

void initExpander(void) {
   
    // set GP0-3 as outputs and GP4-7 as inputs
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x00); //GPIO register is 0x00
    i2c_master_send(0b11110000); //1 is input, 0 is output
    i2c_master_stop();
    
    // enable pull-up resistors on input pins
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x06); // GPPU register is 0x06
    i2c_master_send(0b11110000); //1 enables pull up resistor
    i2c_master_stop();
    
    
    
}