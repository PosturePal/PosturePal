#include "blink.h"

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#define BLINK_MS_2_PERIOD(ms)   (ms * (1000 / Clock_tickPeriod))



Blink::Blink()
{
    _ledCallback = NULL;
    _state = BLINK_STATE_STOPPED;
    _count = BLINK_INFINITELY;
    _begun = false;
}



Blink::Blink(LedCallback_t cb)
{
    _ledCallback = cb;
    _state = BLINK_STATE_STOPPED;
    _count = BLINK_INFINITELY;
    _begun = false;
}



Blink::~Blink()
{

}



void Blink::begin()
{
    if (_begun)
    {
        return;
    }

    _state = BLINK_STATE_STOPPED;
    _count = BLINK_INFINITELY;
    _begun = true;

    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.period = BLINK_MS_2_PERIOD(5);
    clockParams.arg = (UArg) this;

    _clock = Clock_create((Clock_FuncPtr) clockSwHandler,
                          BLINK_MS_2_PERIOD(100),
                          &clockParams,
                          NULL);
}



void Blink::end()
{
    if (!_begun)
    {
        return;
    }

    stop();

    Clock_delete(&_clock);

    _begun = false;
}



void Blink::clockSwHandler(UArg arg)
{
    if (arg == NULL)
    {
        return;
    }

    Blink *self = (Blink *) arg;

    self->loop();
}



void Blink::loop()
{
    if (!_begun)
    {
        return;
    }

    switch (_state)
    {
    case BLINK_STATE_STOPPED:
        // do nothing
        break;



    case BLINK_STATE_LED_ON:

        _state = BLINK_STATE_LED_OFF;

        Clock_stop(_clock);
        Clock_setTimeout(_clock, BLINK_MS_2_PERIOD(_offInterval));
        Clock_start(_clock);

        if (_ledCallback)
        {
            _ledCallback(false);
        }

        if (_count > 0)
        {
            _count--;
        }

        break;



    case BLINK_STATE_LED_OFF:

        if (_count == 0)
        {
            _count = BLINK_INFINITELY;
            _state = BLINK_STATE_STOPPED;
        }
        else
        {
            _state = BLINK_STATE_LED_ON;

            Clock_stop(_clock);
            Clock_setTimeout(_clock, BLINK_MS_2_PERIOD(_onInterval));
            Clock_start(_clock);

            if (_ledCallback)
            {
                _ledCallback(true);
            }
        }

        break;
    }
}



void Blink::start(int32_t count /* = BLINK_INFINITELY */,
                  uint32_t onInterval /* = BLINK_DEFAULT_ON_INTERVAL_MS */,
                  uint32_t offInterval /* = BLINK_DEFAULT_OFF_INTERVAL_MS */)
{
    if (!_begun)
    {
        return;
    }

    _count = count;
    _onInterval = onInterval;
    _offInterval = offInterval;

    _state = BLINK_STATE_LED_ON;

    if (_ledCallback)
    {
        _ledCallback(true);
    }

    Clock_setTimeout(_clock, BLINK_MS_2_PERIOD(_onInterval));
    Clock_start(_clock);
}



void Blink::stop(bool abort /* = false */)
{
    if (!_begun)
    {
        return;
    }

    if ( (_count > 0) && !abort)
    {
        while(_count > 0)
        {
            delay(MIN(_onInterval, _offInterval));
        }
    }
    else
    {
        while (_state == BLINK_STATE_LED_ON)
        {
            delay(MIN(_onInterval, _offInterval));
        }
    }

    Clock_stop(_clock);

    _state = BLINK_STATE_STOPPED;
    _count = BLINK_INFINITELY;
}



void Blink::SetLedCallback(LedCallback_t cb)
{
    stop();
    _ledCallback = cb;
}



bool Blink::GetLedState()
{
    return (_state == BLINK_STATE_LED_ON);
}
