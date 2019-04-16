#ifndef __LSM6DS3_SETTINGS_H__
#define __LSM6DS3_SETTINGS_H__

//This struct holds the settings the driver uses to do calculations
struct SensorSettings
{
public:
    //Gyro settings
    uint8_t gyroEnabled;
    uint8_t gyroInterruptEnable;
    uint16_t gyroRange;
    uint16_t gyroSampleRate;

    uint8_t gyroFifoEnabled;
    uint8_t gyroFifoDecimation;

    //Accelerometer settings
    uint8_t accelEnabled;
    uint8_t accelInterruptEnable;
    uint8_t accelODROff;
    uint16_t accelRange;
    uint16_t accelSampleRate;
    uint16_t accelBandWidth;

    uint8_t accelFifoEnabled;
    uint8_t accelFifoDecimation;

    //Temperature settings
    uint8_t tempEnabled;

    //Non-basic mode settings
    uint8_t commMode;

    //FIFO control data
    uint16_t fifoThreshold;
    int16_t fifoSampleRate;
    uint8_t fifoModeWord;
};


#endif  // End of __LSM6DS3_SETTINGS_H__ definition check
