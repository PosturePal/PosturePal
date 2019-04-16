#ifndef __BLINK_H__
#define __BLINK_H__

#include <stdint.h>
#include "delay.h"

typedef void (* LedCallback_t)(bool);

#define BLINK_DEFAULT_ON_INTERVAL_MS        (500)
#define BLINK_DEFAULT_OFF_INTERVAL_MS       (500)
#define BLINK_INFINITELY                    (-1)


typedef enum
{
    BLINK_STATE_STOPPED = 0,
    BLINK_STATE_LED_ON,
    BLINK_STATE_LED_OFF,
} BlinkState_t;


class Blink
{
public:
    Blink();
    Blink(LedCallback_t cb);
    ~Blink();

public:
    void begin();
    void end();
    void loop();

    void start(int32_t count = BLINK_INFINITELY,
               uint32_t onInterval = BLINK_DEFAULT_ON_INTERVAL_MS,
               uint32_t offInterval = BLINK_DEFAULT_OFF_INTERVAL_MS);

    void stop(bool abort = false);

    void SetLedCallback(LedCallback_t cb);
    bool GetLedState();

protected:
    static void clockSwHandler(UArg arg);

protected:
    LedCallback_t   _ledCallback;
    BlinkState_t    _state;

    uint32_t        _onInterval;
    uint32_t        _offInterval;

    int32_t         _count;
    bool            _begun;

    Clock_Handle    _clock;
};

#endif // __BLINK_H__
