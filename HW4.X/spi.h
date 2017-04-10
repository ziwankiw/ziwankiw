#ifndef SPI_H
#define SPI_H

#include <xc.h>
#include <math.h>

#define PI 3.14159265

int sinewave[1000];
int triwave[1000];

void initSPI1();
void makeSineWave();
void makeTriangleWave();
//char SPI1_IO(char write);

#endif