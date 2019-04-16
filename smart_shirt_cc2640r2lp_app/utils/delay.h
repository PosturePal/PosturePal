#ifndef __DELAY_H__
#define __DELAY_H__

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

#ifdef __cplusplus
extern "C" {
#endif

unsigned long millis();
unsigned long micros();

void yield();

void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

#ifdef __cplusplus
}
#endif

#endif // __DELAY_H__
