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