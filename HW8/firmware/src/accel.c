#include "accel.h"

void initAccel(void) {
   
    // CTRL1_XL register
    // set sample rate 1.66kHz, 2g sensitivity, 100Hz filter
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x10); 
    i2c_master_send(0b10000010);
    i2c_master_stop();
    
    // CTRL2_G register
    // set sample rate 1.66kHz, 100dps sensitivity
    i2c_master_start();
    i2c_master_send(ADDR<<1 | 0);
    i2c_master_send(0x11);
    i2c_master_send(0b10001000);
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char address, unsigned char startreg, unsigned char *bytes, int length) {
    // for accelerometer, address = 0b1101011, startreg = 0x20, length = 14
    i2c_master_start();
    i2c_master_send(address<<1 | 0);
    i2c_master_send(startreg);
    i2c_master_restart();
    i2c_master_send(address<<1 | 1);
    
    int i;
    for(i=1; i<=length; i++) {
        bytes[i] = i2c_master_recv();
        
        if(i<length){
            i2c_master_ack(0);
        }
        else {
            i2c_master_ack(1);
        }
        
        
    }
    
    i2c_master_stop();
}

void reconstructShort(unsigned char *bytes, short *data, int length) {
    
    int i;
    for(i=1; i<=length/2; i++) {
        data[i] = (bytes[i*2] << 8 | bytes[i*2 - 1]);
    }
}