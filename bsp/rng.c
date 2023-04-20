#include "rng.h"

#ifndef IOREG32
#define IOREG32(addr) (*(volatile unsigned long *) (addr))
#endif

#define TASKS_START IOREG32(0x4000D000)
#define TASKS_STOP  IOREG32(0x4000D004)
#define EVENTS_VALRDY IOREG32(0x4000D100)
#define SHORTS IOREG32(0x4000D200)
#define INTSET IOREG32(0x4000D304)
#define INTENCLR IOREG32(0x4000D308)
#define RNG_CONFIG  IOREG32(0x4000d504)
#define VALUE       IOREG32(0x4000D508)

#define BIAS_CORRECTION_ENABLE 1
#define BIAS_CORRECTION_DISABLE 0

void rng_init(void){

    // start rng
    TASKS_START = 1;

    // disable shortcuts between local events and tasks
    SHORTS = 0;

    // bias correction 
    RNG_CONFIG = BIAS_CORRECTION_DISABLE;
}

/*
* This function provides the randomn values from [0, ..., 255] range
* It is obtained from the thermal noise of the processor
* @input:
*        none
* @output:
*        randomn integer value from 0 to 255
*/
int getRngRaw(void){
    return VALUE;
}

/*
* This function provides the randomn values from [min, ..., max-1] range
* @input:
*        min = minimum integer value of the randomn number
*        max = maximum integer value of the randomn number
* @output:
*        randomn number value from min to max-1
*/
int getRng(int min, int max){
    int val = getRngRaw();
    return min + (val % (max));
}

