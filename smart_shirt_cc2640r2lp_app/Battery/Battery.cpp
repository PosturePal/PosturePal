#include "Battery.h"

#include "ti/devices/cc26x0r2/driverlib/aon_batmon.h"


void Battery::begin()
{
    AONBatMonEnable();
}



void Battery::end()
{
    AONBatMonDisable();
}



uint16_t Battery::getCapacity() const
{
    int32_t capacity;

    uint32_t vbatt = getVbatt();

    capacity = ((vbatt - BATTERY_MIN_VOLTAGE) * 100) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);

    if (capacity > 100)
    {
        capacity = 100;
    }
    else if (capacity < 0)
    {
        // impossible, but in any case
        capacity = 0;
    }

    return (uint16_t) capacity;
}



uint16_t Battery::getVbatt() const
{
    return (AONBatMonBatteryVoltageGet() * 1000) / 256;
}



int16_t Battery::getTemperature() const
{
    return AONBatMonTemperatureGetDegC();
}
