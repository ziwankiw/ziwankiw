#ifndef ACCEL_H    /* Guard against multiple inclusion */
#define ACCEL_H

#include <xc.h>
#include "i2c_master_noint.h"

#define ADDR 0b1101011

void initAccel(void);
void I2C_read_multiple(unsigned char address, unsigned char register, unsigned char *data, int length);

#endif