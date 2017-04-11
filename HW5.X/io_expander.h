#ifndef IO_EXPANDER_H__
#define IO_EXPANDER_H__

#define ADDR 0b0100111

#include <xc.h>

void initExpander(void);              // set up I2C 1 as a master, at 100 kHz

#endif