#include <LSM6DS3.h>
#include "stdint.h"
#include "delay.h"
#include "board.h"

//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
#ifdef LSM6DS3_I2C

LSM6DS3::LSM6DS3() :
        LSM6DS3CoreI2C()

#elif defined LSM6DS3_SPI

LSM6DS3::LSM6DS3(uint8_t cs) :
        LSM6DS3CoreSPI(cs)

#endif
{
	//Construct with these default

	settings.gyroEnabled = LSM6DS3_DEFAULT_GYRO_ENABLED;
	settings.gyroInterruptEnable = LSM6DS3_DEFAULT_GYRO_INTERRUPT_ENABLED;
	settings.gyroRange = LSM6DS3_DEFAULT_GYRO_RANGE;
	settings.gyroSampleRate = LSM6DS3_DEFAULT_GYRO_SAMPLE_RATE;
	settings.gyroFifoEnabled = LSM6DS3_DEFAULT_GYRO_FIFO_ENABLED;
	settings.gyroFifoDecimation = LSM6DS3_DEFAULT_GYRO_FIFO_DECIMATION;

	settings.accelEnabled = LSM6DS3_DEFAULT_ACCEL_ENABLED;
	settings.accelInterruptEnable = LSM6DS3_DEFAULT_ACCEL_INTERRUPT_ENABLED;
	settings.accelODROff = LSM6DS3_DEFAULT_ACCEL_ODR_OFF;
	settings.accelRange = LSM6DS3_DEFAULT_ACCEL_RANGE;
	settings.accelSampleRate = LSM6DS3_DEFAULT_ACCEL_SAMPLE_RATE;
	settings.accelBandWidth = LSM6DS3_DEFAULT_ACCEL_BAND_WIDTH;
	settings.accelFifoEnabled = LSM6DS3_DEFAULT_ACCEL_FIFO_ENABLED;
	settings.accelFifoDecimation = LSM6DS3_DEFAULT_ACCEL_FIFO_DECIMATION;

	settings.tempEnabled = LSM6DS3_DEFAULT_TEMP_ENABLED;

	//Select interface mode
	settings.commMode = LSM6DS3_DEFAULT_COMM_MODE;

	//FIFO control data
	settings.fifoThreshold = LSM6DS3_DEFAULT_FIFO_THRESHOLD;
	settings.fifoSampleRate = LSM6DS3_DEFAULT_FIFO_SAMPLE_RATE;
	settings.fifoModeWord = LSM6DS3_DEFAULT_FIFO_MODE_WORD;

	allOnesCounter = LSM6DS3_DEFAULT_ALL_ONES_COUNTER;
	nonSuccessCounter = LSM6DS3_DEFAULT_NON_SUCCESS_COUNTER;
}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t LSM6DS3::begin()
{
    //Begin the inherited core.  This gets the physical wires connected
    status_t returnError = beginCore();

    if (returnError != IMU_SUCCESS)
    {
        return returnError;
    }

    uint8_t result;
    returnError = readRegister(&result, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

    if (returnError != IMU_SUCCESS)
    {
        return returnError;
    }
    else if (result != 0x69)
    {
        return IMU_HW_ERROR;
    }


    returnError = applySettings();

    if (returnError != IMU_SUCCESS)
    {
        return returnError;
    }

    return IMU_SUCCESS;
}



status_t LSM6DS3::applySettings()
{
    status_t returnError = IMU_SUCCESS;

	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	//Setup the accelerometer******************************
	dataToWrite = 0; // Start Fresh!

	if ( settings.accelEnabled == 1)
	{
		//Build config reg
		//First patch in filter bandwidth
		switch (settings.accelBandWidth)
		{
            case LSM6DS3_ACC_BW_50HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
                break;
            case LSM6DS3_ACC_BW_100HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
                break;
            case LSM6DS3_ACC_BW_200HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
                break;
            default:  //set default case to max passthrough
            case LSM6DS3_ACC_BW_400HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
                break;
		}

		//Next, patch in full scale
		switch (settings.accelRange)
		{
            case LSM6DS3_ACC_FS_2:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
                break;
            default:  //set default case to 4g
            case LSM6DS3_ACC_FS_4:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
                break;
            case LSM6DS3_ACC_FS_8:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
                break;
            case LSM6DS3_ACC_FS_16:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
                break;
		}

		//Lastly, patch in accelerometer ODR
		switch (settings.accelSampleRate)
		{
            case LSM6DS3_ODR_13HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
                break;
            case LSM6DS3_ODR_26HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
                break;
            default:  //Set default to 52
            case LSM6DS3_ODR_52HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
                break;
            case LSM6DS3_ODR_104HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
                break;
            case LSM6DS3_ODR_208HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
                break;
            case LSM6DS3_ODR_416HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
                break;
            case LSM6DS3_ODR_833HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
                break;
            case LSM6DS3_ODR_1666HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
                break;
            case LSM6DS3_ODR_3330HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
                break;
            case LSM6DS3_ODR_6660HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
                break;
            case LSM6DS3_ODR_13330HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
                break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}

	//Now, write the patched together data
	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

	if ( settings.accelODROff == 1)
	{
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}

	writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);




	//Setup the gyroscope**********************************************

	dataToWrite = 0; //Start Fresh!

	if ( settings.gyroEnabled == 1)
	{
		//Build config reg
		//First, patch in full scale
		switch (settings.gyroRange)
		{
            case LSM6DS3_GYRO_FS_125:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
                break;
            case LSM6DS3_GYRO_FS_245:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
                break;
            case LSM6DS3_GYRO_FS_500:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
                break;
            case LSM6DS3_GYRO_FS_1000:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
                break;
            default:  //Default to full 2000DPS range
            case LSM6DS3_GYRO_FS_2000:
                dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
                break;
		}

		//Lastly, patch in gyro ODR
		switch (settings.gyroSampleRate)
		{
            case LSM6DS3_ODR_13HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
                break;
            case LSM6DS3_ODR_26HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
                break;
            case LSM6DS3_ODR_52HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
                break;
            default:  //Set default to 104
            case LSM6DS3_ODR_104HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
                break;
            case LSM6DS3_ODR_208HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
                break;
            case LSM6DS3_ODR_416HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
                break;
            case LSM6DS3_ODR_833HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
                break;
            case LSM6DS3_ODR_1666HZ:
                dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
                break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}

	//Write the byte
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

	//Setup the internal temperature sensor
	if ( settings.tempEnabled == 1)
	{
	    // TODO
	}



	dataToWrite = 0;

    if ( (settings.accelEnabled == 1) && (settings.accelInterruptEnable == 1) )
    {
        dataToWrite |= LSM6DS3_ACC_GYRO_INT1_DRDY_XL_ENABLED;
    }

	if ( (settings.gyroEnabled == 1) && (settings.gyroInterruptEnable == 1) )
	{
	    dataToWrite |= LSM6DS3_ACC_GYRO_INT1_DRDY_G_ENABLED;
	}

	if (dataToWrite != 0)
	{
	    writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, dataToWrite);
	}

	writeRegister(LSM6DS3_ACC_GYRO_ORIENT_CFG_G, 0x01 << 7); //  pulsed DATA READY

	return returnError;
}


status_t LSM6DS3::enableAcc(bool enable)
{
    settings.accelEnabled = enable;
    return applySettings();
}

status_t LSM6DS3::enableGyro(bool enable)
{
    settings.gyroEnabled = enable;
    return applySettings();
}

status_t LSM6DS3::enableAccInterrupt(bool enable)
{
    settings.accelInterruptEnable = enable;
    return applySettings();
}

status_t LSM6DS3::enableGyroInterrupt(bool enable)
{
    settings.gyroInterruptEnable = enable;
    return applySettings();
}

status_t LSM6DS3::setAccGyroODR(uint16_t odr)
{
    settings.accelSampleRate = odr;
    settings.gyroSampleRate = odr;
    return applySettings();
}

status_t LSM6DS3::setAccBW(uint16_t bw)
{
    settings.accelBandWidth = bw;
    return applySettings();
}

status_t LSM6DS3::setAccRange(uint16_t range)
{
    settings.accelRange = range;
    return applySettings();
}

status_t LSM6DS3::setAccODR(uint16_t odr)
{
    settings.accelSampleRate = odr;
    return applySettings();
}

status_t LSM6DS3::setGyroRange(uint16_t range)
{
    settings.gyroRange = range;
    return applySettings();
}

status_t LSM6DS3::setGyroODR(uint16_t odr)
{
    settings.gyroSampleRate = odr;
    return applySettings();
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to stop the IMU
//
//****************************************************************************//
status_t LSM6DS3::end()
{
    return endCore();
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LSM6DS3::readRawAccelX(void)
{
	int16_t output;

	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_XL );

	if ( errorLevel != IMU_SUCCESS )
	{
		if ( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}

	return output;
}

float LSM6DS3::readFloatAccelX(void)
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int32_t LSM6DS3::readIntAccelX(void)
{
    int32_t output = calcAccelInt(readRawAccelX());
    return output;
}

int16_t LSM6DS3::readRawAccelY(void)
{
	int16_t output;

	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_XL );

	if ( errorLevel != IMU_SUCCESS )
	{
		if ( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}

	return output;
}

float LSM6DS3::readFloatAccelY(void)
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int32_t LSM6DS3::readIntAccelY(void)
{
    int32_t output = calcAccelInt(readRawAccelY());
    return output;
}

int16_t LSM6DS3::readRawAccelZ(void)
{
	int16_t output;

	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_XL );

	if ( errorLevel != IMU_SUCCESS )
	{
		if ( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}

	return output;
}

float LSM6DS3::readFloatAccelZ(void)
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

int32_t LSM6DS3::readIntAccelZ(void)
{
    int32_t output = calcAccelInt(readRawAccelZ());
    return output;
}

float LSM6DS3::calcAccel( int16_t input )
{
	float output = ( (float) calcAccelInt(input) ) / 1000.0;
	return output;
}

int32_t LSM6DS3::calcAccelInt(int16_t input)
{
    return ( ((int32_t) input) * ((int32_t) getAccelRangeMultiplier())) / 1000;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t LSM6DS3::readRawGyroX(void)
{
	int16_t output;

	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_G );

	if ( errorLevel != IMU_SUCCESS )
	{
		if ( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}

	return output;
}

float LSM6DS3::readFloatGyroX(void)
{
	float output = calcGyro(readRawGyroX());
	return output;
}

int32_t LSM6DS3::readIntGyroX(void)
{
    int32_t output = calcGyroInt(readRawGyroX());
    return output;
}

int16_t LSM6DS3::readRawGyroY(void)
{
	int16_t output;

	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_G );

	if ( errorLevel != IMU_SUCCESS )
	{
		if ( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}

	return output;
}

float LSM6DS3::readFloatGyroY(void)
{
	float output = calcGyro(readRawGyroY());
	return output;
}

int32_t LSM6DS3::readIntGyroY(void)
{
    int32_t output = calcGyroInt(readRawGyroY());
    return output;
}

int16_t LSM6DS3::readRawGyroZ(void)
{
	int16_t output;

	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_G );

	if ( errorLevel != IMU_SUCCESS )
	{
		if ( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}

	return output;
}

float LSM6DS3::readFloatGyroZ(void)
{
	float output = calcGyro(readRawGyroZ());
	return output;
}

int32_t LSM6DS3::readIntGyroZ(void)
{
    int32_t output = calcGyroInt(readRawGyroZ());
    return output;
}

float LSM6DS3::calcGyro( int16_t input )
{
	float output = ((float) calcGyroInt(input)) / 1000.0;
	return output;
}

int32_t LSM6DS3::calcGyroInt(int16_t input)
{
    int64_t multiplier = getGyroRangeMultiplier();

    int32_t output = (int32_t) (( ((int64_t) input) * multiplier) / 1000);

    return output;
}

uint32_t LSM6DS3::getAccelRangeMultiplier()
{
    switch (settings.accelRange)
    {
        case LSM6DS3_ACC_FS_2:  return  61; // 0.061 x 1000
        case LSM6DS3_ACC_FS_4:  return 122; // 0.122 x 1000
        case LSM6DS3_ACC_FS_8:  return 244; // 0.244 x 1000
        case LSM6DS3_ACC_FS_16: return 488; // 0.488 x 1000
    }

    return 1000;
}

uint32_t LSM6DS3::getGyroRangeMultiplier()
{
    switch (settings.gyroRange)
    {
        case LSM6DS3_GYRO_FS_125:  return  4375; //  4.375 x 1000
        case LSM6DS3_GYRO_FS_245:  return  8750; //  8.750 x 1000
        case LSM6DS3_GYRO_FS_500:  return 17500; // 17.5 x 1000
        case LSM6DS3_GYRO_FS_1000: return 35000; // 35.0 x 1000
        case LSM6DS3_GYRO_FS_2000: return 70000; // 70.0 x 1000
    }

    return 1000;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LSM6DS3::readRawTemp(void)
{
	int16_t output;
	readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUT_TEMP_L );
	return output;
}  

float LSM6DS3::readTempC(void)
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;

}

float LSM6DS3::readTempF(void)
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;

}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LSM6DS3::fifoBegin(void)
{
	//CONFIGURE THE VARIOUS FIFO SETTINGS
	//
	//
	//This section first builds a bunch of config words, then goes through
	//and writes them all.

	//Split and mask the threshold
	uint8_t thresholdLByte = settings.fifoThreshold & 0x00FF;
	uint8_t thresholdHByte = (settings.fifoThreshold & 0x0F00) >> 8;
	//Pedo bits not configured (ctl2)

	//CONFIGURE FIFO_CTRL3
	uint8_t tempFIFO_CTRL3 = 0;
	if (settings.gyroFifoEnabled == 1)
	{
		//Set up gyro stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (settings.gyroFifoDecimation & 0x07) << 3;

	}
	if (settings.accelFifoEnabled == 1)
	{
		//Set up accelerometer stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (settings.accelFifoDecimation & 0x07);
	}

	//CONFIGURE FIFO_CTRL4  (nothing for now-- sets data sets 3 and 4
	uint8_t tempFIFO_CTRL4 = 0;


	//CONFIGURE FIFO_CTRL5
	uint8_t tempFIFO_CTRL5 = 0;
	switch (settings.fifoSampleRate) {
	default:  //set default case to 10Hz(slowest)
	case 10:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz;
		break;
	case 25:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz;
		break;
	case 50:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz;
		break;
	case 100:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
		break;
	case 200:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
		break;
	case 400:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz;
		break;
	case 800:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz;
		break;
	case 1600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz;
		break;
	case 3300:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz;
		break;
	case 6600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz;
		break;
	}
	//Hard code the fifo mode here:
	tempFIFO_CTRL5 |= settings.fifoModeWord = 6;  //set mode:

	//Write the data
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
	//Serial.println(thresholdLByte, HEX);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);
	//Serial.println(thresholdHByte, HEX);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

}
void LSM6DS3::fifoClear(void)
{
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x1000 ) == 0 ) {
		fifoRead();
	}

}
int16_t LSM6DS3::fifoRead(void)
{
	//Pull the last data from the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);
	tempAccumulator |= ((uint16_t)tempReadByte << 8);

	return tempAccumulator;
}

uint16_t LSM6DS3::fifoGetStatus(void)
{
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS1);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS2);
	tempAccumulator |= (tempReadByte << 8);

	return tempAccumulator;  

}
void LSM6DS3::fifoEnd(void)
{
	// turn off the fifo
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_STATUS1, 0x00);  //Disable
}

