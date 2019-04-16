#include <ti/drivers/GPIO.h>
#include <string.h> // because of memcpy
#include "app2bleBridge.h"
#include "smart_shirt.h"
#include "ble_services.h"
#include "board.h"

/*
 * bleSetAccGyroData - Sets Acc/Gyro sensor data
 *
 *    timestamp - timestamp in microseconds
 *    ax - acceleration X axis
 *    ay - acceleration Y axis
 *    az - acceleration Z axis
 *    gx - rotation X axis
 *    gy - rotation Y axis
 *    gz - rotation Z axis
 */
void bleSetAccGyroData(uint32_t timestamp,
                       BleAccGyroData_t * accGyroData,
                       uint8_t *data)
{
    size_t offset = 0;
    memcpy(&data[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);

    memcpy(&data[offset], &accGyroData->ax, sizeof(accGyroData->ax));
    offset += sizeof(accGyroData->ax);

    memcpy(&data[offset], &accGyroData->ay, sizeof(accGyroData->ay));
    offset += sizeof(accGyroData->ay);

    memcpy(&data[offset], &accGyroData->az, sizeof(accGyroData->az));
    offset += sizeof(accGyroData->az);


    memcpy(&data[offset], &accGyroData->gx, sizeof(accGyroData->gx));
    offset += sizeof(accGyroData->gx);

    memcpy(&data[offset], &accGyroData->gy, sizeof(accGyroData->gy));
    offset += sizeof(accGyroData->gy);

    memcpy(&data[offset], &accGyroData->gz, sizeof(accGyroData->gz));
    offset += sizeof(accGyroData->gz);

    uint32_t multipliers = 0;

    multipliers |= (accGyroData->am <<  0) & 0x00000FFF; // 12 bit
    multipliers |= (accGyroData->gm << 12) & 0xFFFFF000; // 20 bit;

    memcpy(&data[offset], &multipliers, sizeof(multipliers));
    offset += sizeof(multipliers);

    // TODO push the data in queue
    user_SetParameter(SENSOR_SERVICE_SERV_UUID, SS_DATA_ID, (void *) data, SS_DATA_LEN);
}



/*
 * bleSetBatteryVoltage - Sets battery voltage parameter
 *
 *    voltage - (current) battery voltage to be set
 */
void bleSetBatteryVoltage(uint16_t voltage)
{
    user_SetParameter(BATTERY_SERVICE_SERV_UUID,
                      BS_VBATT_ID,
                      (void *) &voltage,
                      BS_VBATT_LEN);
}



/*
 * bleSetBatteryMaxVoltage - Sets battery maximum voltage parameter
 *
 *    voltage - (current) battery voltage to be set
 */
void bleSetBatteryMaxVoltage(uint16_t voltage)
{
    user_SetParameter(BATTERY_SERVICE_SERV_UUID,
                      BS_MAX_VBATT_ID,
                      (void *) &voltage,
                      BS_MAX_VBATT_LEN);
}



/*
 * bleSetBatteryCapacity - Sets battery maximum voltage parameter
 *
 *    capacity - battery capacity to be set
 */
void bleSetBatteryCapacity(uint16_t capacity)
{
    user_SetParameter(BATTERY_SERVICE_SERV_UUID,
                      BS_CBATT_ID,
                      (void *) &capacity,
                      BS_CBATT_LEN);
}



/*
 * bleSetBatteryTemperature - Sets ambient temperature parameter
 *
 *    temp - temperature in degrees Centigrate to be set
 */
void bleSetBatteryTemperature(int16_t temp)
{
    user_SetParameter(BATTERY_SERVICE_SERV_UUID,
                      BS_TEMP_ID,
                      (void *) &temp,
                      BS_TEMP_LEN);
}



/*
 * bleSetVibrationState - Sets VIBRATION state
 *
 *    state - 0: off / 1: on
 */
void bleSetVibrationState(bool state)
{
    user_SetParameter(UI_SERVICE_SERV_UUID,
                      US_VIBRATION_ID,
                      (void *) &state,
                      US_VIBRATION_LEN);
}



/*
 * bleSetLedState - Sets LED state
 *
 *    state - 0: off / 1: on
 */
void bleSetLedState(bool state)
{
    user_SetParameter(UI_SERVICE_SERV_UUID,
                      US_LED_ID,
                      (void *) &state,
                      US_LED_LEN);
}



/*
 * bleSetButtonState - Sets BUTTON state
 *
 *    state - 0: released / 1: pushed
 */
void bleSetButtonState(uint8_t state)
{
    user_SetParameter(UI_SERVICE_SERV_UUID,
                      US_BUTTON_ID,
                      (void *) &state,
                      US_BUTTON_LEN);
}



void bleSetAccEnable(bool enable)
{
    user_SetParameter(SENSOR_SERVICE_SERV_UUID,
                      SS_ACC_ENABLE_ID,
                      (void *) &enable,
                      SS_ACC_ENABLE_LEN);
}



void bleSetAccODR(uint32_t odr)
{
    user_SetParameter(SENSOR_SERVICE_SERV_UUID,
                      SS_ACC_ODR_ID,
                      (void *) &odr,
                      SS_ACC_ODR_LEN);
}



void bleSetAccFS(uint32_t fs)
{
    user_SetParameter(SENSOR_SERVICE_SERV_UUID,
                      SS_ACC_FS_ID,
                      (void *) &fs,
                      SS_ACC_FS_LEN);
}



void bleSetGyroEnable(bool enable)
{
    user_SetParameter(SENSOR_SERVICE_SERV_UUID,
                      SS_GYRO_ENABLE_ID,
                      (void *) &enable,
                      SS_GYRO_ENABLE_LEN);
}



void bleSetGyroODR(uint32_t odr)
{
    user_SetParameter(SENSOR_SERVICE_SERV_UUID,
                      SS_GYRO_ODR_ID,
                      (void *) &odr,
                      SS_GYRO_ODR_LEN);
}



void bleSetGyroFS(uint32_t fs)
{
    user_SetParameter(SENSOR_SERVICE_SERV_UUID,
                      SS_GYRO_FS_ID,
                      (void *) &fs,
                      SS_GYRO_FS_LEN);
}
