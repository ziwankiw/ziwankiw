#include "io_expander.h"
#include "i2c_master_noint.h"

void initExpander(void) {
   
    // set GP0-3 as outputs and GP4-7 as inputs
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x00); //IODIR register is 0x00
    i2c_master_send(0b11110000); //1 is input, 0 is output
    i2c_master_stop();
    
    // configuration register
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x05); //IOCON register is 0x05
    i2c_master_send(0b00010100);
    i2c_master_stop();
    
    // enable pull-up resistors on input pins
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x06); // GPPU register is 0x06
    i2c_master_send(0b11110000); //1 enables pull up resistor on inputs
    i2c_master_stop();
}

void setExpander(char pin, char level){
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x09); // GPIO register is 0x09
    i2c_master_send(level<<pin); //level of 1 sets output to high
    i2c_master_stop();
}

unsigned char getExpander() {
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x09); // GPIO register is 0x09
    i2c_master_restart();
    i2c_master_send(ADDR<<1 | 1);
    unsigned char byte = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return byte;
}
