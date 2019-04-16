#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <stdint.h>

#define BATTERY_MIN_VOLTAGE     (1800)
#define BATTERY_MAX_VOLTAGE     (3000)

class Battery
{
public:
    Battery() :
        _battMaxVoltage(BATTERY_MAX_VOLTAGE)
    { }

    ~Battery() { }

public:

    void begin();
    void end();

    uint16_t getCapacity() const;
    uint16_t getVbatt() const;

    void setVbattMax(uint16_t vbattmax) { _battMaxVoltage = vbattmax; }
    uint16_t getVbattMax() const { return _battMaxVoltage; }

    int16_t getTemperature() const;

private:
    uint16_t _battMaxVoltage;
};

#endif // __BATTERY_H__
