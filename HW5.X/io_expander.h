#ifndef IO_EXPANDER_H__
#define IO_EXPANDER_H__

#include <xc.h>

#define ADDR 0b0100111

void initExpander(void);
void setExpander(char pin, char level);
unsigned char getExpander();

#endif
