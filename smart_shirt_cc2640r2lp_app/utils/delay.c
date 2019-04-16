#include "delay.h"

/*
 *  ======== millis ========
 */
unsigned long millis()
{
    return (Clock_getTicks()/(1000/Clock_tickPeriod));
}

/*
 *  ======== micros ========
 */
unsigned long micros()
{
    return Clock_getTicks() * Clock_tickPeriod;
}

void yield()
{
    Task_yield();
}

/*
 *  ======== delay ========
 */
void delay(unsigned long ms)
{
    if (ms == 0) {
        Task_yield();
        return;
    }

#ifdef DELAY_WITH_NOPS
    while(ms--)
    {
        Task_yield();
        delayMicroseconds(1000);
    }
#else
    Task_sleep(ms*(1000/Clock_tickPeriod));
#endif
}

/*
 *  ======== delayMicroseconds ========
 *  Delay for the given number of microseconds
 */
void delayMicroseconds(unsigned int us)
{

    int time = 0;

    if (us <= 5) {
        time = (((us * 42) + 5)/10); // 4.2
    }
    else if (us <= 10) {
        time = (((us * 48) + 5)/10); // 4.8
    }
    else if (us <= 20)
    {
        time = (((us * 50) + 5)/10); // 5.0
    }
    else if (us <= 50)
    {
        time = (((us * 52) + 5)/10); // 5.2
    }
    else if (us <= 500)
    {
        time = (((us * 54) + 5)/10); // 5.4
    }
    else
    {
        unsigned long tmp = micros() + us;

        while (micros() < tmp)
        {
            Task_yield();
        }
		return;
    }

    while (time--)
    {
        asm("   nop");
    }
}
