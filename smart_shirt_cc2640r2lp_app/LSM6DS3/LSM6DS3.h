#ifndef __LSM6DS3_H__
#define __LSM6DS3_H__

#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <stdint.h>
#include "LSM6DS3_Core_SPI.h"
#include "LSM6DS3_Core_I2C.h"
#include "LSM6DS3_Registers.h"
#include "LSM6DS3_Settings.h"


//#define LSM6DS3_SPI
#define LSM6DS3_I2C

typedef enum
{
    LSM6DS3_ODR_13HZ    = 13,   // 76.9 ms
    LSM6DS3_ODR_26HZ    = 26,   // 38.4 ms
    LSM6DS3_ODR_52HZ    = 52,   // 19.2 ms
    LSM6DS3_ODR_104HZ   = 104,  //  9.6 ms
    LSM6DS3_ODR_208HZ   = 208,  //  4.8 ms
    LSM6DS3_ODR_416HZ   = 416,  //  2.4 ms
    LSM6DS3_ODR_833HZ   = 833,  //  1.2 ms
    LSM6DS3_ODR_1666HZ  = 1666, //  0,6 ms
    LSM6DS3_ODR_3330HZ  = 3330, //  0,3 ms
    LSM6DS3_ODR_6660HZ  = 6660, //  0,15 ms
    LSM6DS3_ODR_13330HZ = 13330,//  0,075 ms
} LSM6DS3_ODR_t;



typedef enum
{
    LSM6DS3_ACC_FS_2    = 2,
    LSM6DS3_ACC_FS_4    = 4,
    LSM6DS3_ACC_FS_8    = 8,
    LSM6DS3_ACC_FS_16   = 16
} LSM6DS3_ACC_FS_t;



typedef enum
{
    LSM6DS3_GYRO_FS_125     = 125,
    LSM6DS3_GYRO_FS_245     = 245,
    LSM6DS3_GYRO_FS_500     = 500,
    LSM6DS3_GYRO_FS_1000    = 1000,
    LSM6DS3_GYRO_FS_2000    = 2000
} LSM6DS3_GYRO_FS_t;



typedef enum
{
    LSM6DS3_ACC_BW_50HZ     = 50,
    LSM6DS3_ACC_BW_100HZ    = 100,
    LSM6DS3_ACC_BW_200HZ    = 200,
    LSM6DS3_ACC_BW_400HZ    = 400,
} LSM6DS3_ACC_BW_t;




#define LSM6DS3_ACC_ODR_MIN     (LSM6DS3_ODR_13HZ)
#define LSM6DS3_ACC_ODR_MAX     (LSM6DS3_ODR_13330HZ)

#define LSM6DS3_GYRO_ODR_MIN    (LSM6DS3_ODR_13HZ)
#define LSM6DS3_GYRO_ODR_MAX    (LSM6DS3_ODR_1666HZ)

#define LSM6DS3_ACC_FS_MIN      (LSM6DS3_ACC_FS_2)
#define LSM6DS3_ACC_FS_MAX      (LSM6DS3_ACC_FS_16)

#define LSM6DS3_GYRO_FS_MIN     (LSM6DS3_GYRO_FS_125)
#define LSM6DS3_GYRO_FS_MAX     (LSM6DS3_GYRO_FS_2000)

#define LSM6DS3_ACC_BW_MIN      (LSM6DS3_ACC_BW_50HZ)
#define LSM6DS3_ACC_BW_MAX      (LSM6DS3_ACC_BW_400HZ)


#define LSM6DS3_DEFAULT_ACCEL_ENABLED           (1)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_ACCEL_INTERRUPT_ENABLED (1)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_ACCEL_ODR_OFF           (1)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_ACCEL_RANGE             (LSM6DS3_ACC_FS_8)    // Max G force readable.  Can be: 2, 4, 8, 16
#define LSM6DS3_DEFAULT_ACCEL_SAMPLE_RATE       (LSM6DS3_ODR_52HZ)    // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
#define LSM6DS3_DEFAULT_ACCEL_BAND_WIDTH        (LSM6DS3_ACC_BW_MAX)   // Hz.  Can be: 50, 100, 200, 400;
#define LSM6DS3_DEFAULT_ACCEL_FIFO_ENABLED      (0)     // Set to include accelerometer in the FIFO
#define LSM6DS3_DEFAULT_ACCEL_FIFO_DECIMATION   (1)     // Set 1 for on /1

#define LSM6DS3_DEFAULT_GYRO_ENABLED            (1)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_GYRO_INTERRUPT_ENABLED  (1)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_GYRO_RANGE              (LSM6DS3_GYRO_FS_2000)  // Max deg/s.  Can be: 125, 245, 500, 1000, 2000
#define LSM6DS3_DEFAULT_GYRO_SAMPLE_RATE        (LSM6DS3_ODR_52HZ)   // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
#define LSM6DS3_DEFAULT_GYRO_FIFO_ENABLED       (0)     // Set to include gyro in FIFO
#define LSM6DS3_DEFAULT_GYRO_FIFO_DECIMATION    (1)     // Set 1 for on /1

#define LSM6DS3_DEFAULT_TEMP_ENABLED            (0)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_COMM_MODE               (1)     // Can be modes 1, 2 or 3

#define LSM6DS3_DEFAULT_FIFO_THRESHOLD          (0)     // Can be 0 to 4096 (16 bit bytes)
#define LSM6DS3_DEFAULT_FIFO_SAMPLE_RATE        (10)    // Default 10Hz
#define LSM6DS3_DEFAULT_FIFO_MODE_WORD          (0)     // Default OFF

#define LSM6DS3_DEFAULT_ALL_ONES_COUNTER        (0)     // Can be 0 or 1
#define LSM6DS3_DEFAULT_NON_SUCCESS_COUNTER     (0)     // Can be 0 or 1

//This is the highest level class of the driver.
//
//  class LSM6DS3 inherits the core and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

class LSM6DS3 : public LSM6DS3CoreI2C
{
public:
	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
    LSM6DS3();
	// LSM6DS3(uint8_t cs) : LSM6DS3CoreSPI(cs);
	~LSM6DS3() { }
	
public:
	// Configuration routines
	status_t enableAcc(bool enable);
    bool getAccEnabled() const { return settings.accelEnabled; }

	status_t enableGyro(bool enable);
    bool getGyroEnabled() const { return settings.gyroEnabled; }

	status_t setAccGyroODR(uint16_t odr);

	status_t setAccRange(uint16_t range);
	uint16_t getAccRange() const { return settings.accelRange; }

	status_t setAccODR(uint16_t odr);
	uint16_t getAccODR() const { return settings.accelSampleRate; }

	status_t setGyroRange(uint16_t range);
	uint16_t getGyroRange() const { return settings.gyroRange; }

	status_t setGyroODR(uint16_t odr);
	uint16_t getGyroODR() const { return settings.gyroSampleRate; }

	//Call to apply SensorSettings
	status_t begin(void);
	status_t end(void);

	//Returns the raw bits from the sensor cast as 16-bit signed integers
	int16_t readRawAccelX(void);
	int16_t readRawAccelY(void);
	int16_t readRawAccelZ(void);
	int16_t readRawGyroX(void);
	int16_t readRawGyroY(void);
	int16_t readRawGyroZ(void);

	//Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX(void);
	float readFloatAccelY(void);
	float readFloatAccelZ(void);
	float readFloatGyroX(void);
	float readFloatGyroY(void);
	float readFloatGyroZ(void);

	int32_t readIntAccelX(void);
	int32_t readIntAccelY(void);
	int32_t readIntAccelZ(void);

	int32_t readIntGyroX(void);
	int32_t readIntGyroY(void);
	int32_t readIntGyroZ(void);

	//Temperature related methods
	int16_t readRawTemp(void);
	float readTempC(void);
	float readTempF(void);

	//FIFO stuff
	void fifoBegin(void);
	void fifoClear(void);
	int16_t fifoRead(void);
	uint16_t fifoGetStatus(void);
	void fifoEnd(void);
	
	float calcGyro(int16_t);
	float calcAccel(int16_t);

    int32_t calcGyroInt(int16_t);
    int32_t calcAccelInt(int16_t);

    uint32_t getAccelRangeMultiplier();
    uint32_t getGyroRangeMultiplier();


protected:
    status_t setAccBW(uint16_t bw);
    status_t enableAccInterrupt(bool enable);
    status_t enableGyroInterrupt(bool enable);
    status_t applySettings();

protected:
    //IMU settings
    SensorSettings settings;

    //Error checking
    uint16_t allOnesCounter;
    uint16_t nonSuccessCounter;

private:

};

#endif  // End of __LSM6DS3_H__ definition check
