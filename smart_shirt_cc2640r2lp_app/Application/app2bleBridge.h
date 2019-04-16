#ifndef __APP_2_BLE_BRIDGE_H__
#define __APP_2_BLE_BRIDGE_H__

#include "smart_shirt.h"
#include "icall_ble_api.h"
#include <icall.h>
#include <osal_snv.h>
#include <peripheral.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint32_t am;
    uint32_t gm;
} BleAccGyroData_t;

void bleSetAccGyroData(uint32_t timestamp,
                       BleAccGyroData_t *accGyroData,
                       uint8_t *data); // OK

void bleSetBatteryVoltage(uint16_t voltage); // OK
void bleSetBatteryMaxVoltage(uint16_t voltage); // TODO use variable
void bleSetBatteryCapacity(uint16_t capacity); // OK
void bleSetBatteryTemperature(int16_t temp); // OK

void bleSetVibrationState(bool state); // OK
void bleSetLedState(bool state); // OK
void bleSetButtonState(uint8_t state); // OK


void bleSetAccEnable(bool enable);
void bleSetAccODR(uint32_t odr);
void bleSetAccFS(uint32_t fs);

void bleSetGyroEnable(bool enable);
void bleSetGyroODR(uint32_t odr);
void bleSetGyroFS(uint32_t fs);

#endif // __APP_2_BLE_BRIDGE_H__
