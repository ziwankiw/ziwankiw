#ifndef SPI_H
#define SPI_H

#include <xc.h>
#include <math.h>

#define CS LATBbits.LATB7
#define PI 3.14159265

unsigned char sinewave[1000];
unsigned char triwave[1000];

void initSPI1();
void makeSineWave();
void makeTriangleWave();
void setVoltage(unsigned char channel, unsigned char voltage);
char SPI1_IO(unsigned char write);

#endif