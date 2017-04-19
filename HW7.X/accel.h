#ifndef ACCEL_H    /* Guard against multiple inclusion */
#define ACCEL_H

#include <xc.h>
#include "i2c_master_noint.h"

#define ADDR 0b1101011

void initAccel(void);
void I2C_read_multiple(unsigned char address, unsigned char startreg, unsigned char *bytes, int length);
void reconstructShort(unsigned char *bytes, short *data, int length);

#endif