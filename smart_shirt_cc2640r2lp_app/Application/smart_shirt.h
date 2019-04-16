#ifndef __SMART_SHIRT_H__
#define __SMART_SHIRT_H__

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/devices/cc26x0r2/inc/hw_memmap.h>
#include <ti/devices/cc26x0r2/inc/hw_fcfg1.h>


#define SMSH_SKIP_VALUES_ON_FS_CHANGE   (2)
#define USE_MADGWICK_AHRS               (0)

#define IMU_GYRO_CALLIBRATION_X         (-2.9256)
#define IMU_GYRO_CALLIBRATION_Y         (-5.2877)
#define IMU_GYRO_CALLIBRATION_Z         (-1.6566)

void SmartShirt_createTask(void);

void user_Init();
void user_AdvertisingEvent();
void user_ConnectedEvent();
void user_DisconnectedEvent();
void user_ConnectionTimeoutEvent();

void user_ButtonCallback(uint_least8_t index);
void user_LsmCallback(uint_least8_t index);
void user_ChargeCallback(uint_least8_t index);

void user_Vibration(bool on);
void user_Led(bool on);

void user_LsmStart();
void user_LsmStop();

void user_SetParameter(uint16_t service, uint8_t param, void *data, size_t len);

void user_connTimeoutClockSwiHandler(UArg paramID);
void user_dataUpdateClockSwiHandler(UArg paramID);
void user_lsmDataUpdateClockSwiHandler(UArg paramID);
void user_powerOffClockSwiHandler(UArg paramID);

void user_ShutDown();


void user_createTask();

void user_taskFxn(UArg a0, UArg a1);

void user_SetParameter(uint16_t service, uint8_t param, void *data, size_t len);


// Setters
void user_setBatteryMaxVoltage(uint16_t vbattMax);

void user_setAccGyroODR(uint32_t odr);

void user_setAccEnable(bool enable);
void user_setAccODR(uint32_t odr);
void user_setAccFS(uint16_t fs);

void user_setGyroEnable(bool enable);
void user_setGyroODR(uint32_t odr);
void user_setGyroFS(uint16_t fs);

// Getters
bool user_getAccEnable();
uint32_t user_getAccODR();
uint16_t user_getAccFS();

bool user_getGyroEnable();
uint32_t user_getGyroODR();
uint16_t user_getGyroFS();

uint16_t user_getBatteryVoltage();
uint16_t user_getBatteryMaxVoltage();
uint16_t user_getBatteryCapacity();
int16_t user_getBatteryTemperature();

bool user_getButtonState();
bool user_getVibrationState();
bool user_getLedState();

void user_readAndSendLsmData();

#endif /* __SMART_SHIRT_H__ */
