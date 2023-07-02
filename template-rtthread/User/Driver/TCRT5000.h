#ifndef __TCRT5000_h
#define __TCRT5000_h 

#include <stdint.h>
#include <stdbool.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

//p5.5->analog

#define BITS  16384          //2^13
#define THRESHOLD1 1
#define THRESHOLD2 2

extern float adcV;
extern float Voltage;

void init_TCRT();
void read_TCRT();

#endif // !__TCRT5000.h