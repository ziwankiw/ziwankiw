#ifndef SPI_H
#define SPI_H

#include <xc.h>
#include <math.h>

#define CS LATBbits.LATB7
#define PI 3.14159265

char sinewave[1000];
char triwave[1000];

void initSPI1();
void makeSineWave();
void makeTriangleWave();
void setVoltage(char channel, char voltage);
char SPI1_IO(char write);

#endif