#include <LSM6DS3.h>
#include "stdint.h"
#include "delay.h"
#include "board.h"

//****************************************************************************//
//
//  LSM6DS3CoreI2C functions.
//
//  Construction arguments:
//  uint8_t cs: chip select pin number
//
//
//****************************************************************************//
LSM6DS3CoreI2C::LSM6DS3CoreI2C(uint8_t address) :
    _i2c(NULL),
    _address(address)
{
}

status_t LSM6DS3CoreI2C::beginCore(void)
{
    if (_i2c == NULL)
    {
        // Initialize the SPI driver.
        I2C_init();

        I2C_Params i2cParams;

        // Create a SPI with data processing off.
        I2C_Params_init(&i2cParams);

        i2cParams.transferMode = I2C_MODE_BLOCKING;
        i2cParams.bitRate = I2C_100kHz;

        _i2c = I2C_open(CC2640R2_LAUNCHXL_I2C0, &i2cParams);

        if (_i2c == NULL)
        {
            // I2C_open() failed
            return IMU_HW_ERROR;
        }
    }

    //Spin for a few ms
    delay(100); // TODO is it needed ???

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

	if (readCheck != 0x69)
	{
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;
}

//****************************************************************************//
//
//  LSM6DS3CoreI2C functions.
//
//  Construction arguments:
//
//
//****************************************************************************//
status_t LSM6DS3CoreI2C::endCore(void)
{
    I2C_close(_i2c);

    _i2c = NULL;

    return IMU_SUCCESS;
}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LSM6DS3CoreI2C::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
    if (!_i2c)
    {
        return IMU_GENERIC_ERROR;
    }

    I2C_Transaction i2cTransaction;

    i2cTransaction.slaveAddress = _address;

    i2cTransaction.writeBuf = &offset;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = outputPointer;
    i2cTransaction.readCount = length;

    if (!I2C_transfer(_i2c, &i2cTransaction))
    {
        return IMU_GENERIC_ERROR;
    }

	return IMU_SUCCESS;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS3CoreI2C::readRegister(uint8_t* outputPointer, uint8_t offset)
{
    return readRegisterRegion(outputPointer, offset, 1);
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS3CoreI2C::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
    if (!_i2c)
    {
        return IMU_GENERIC_ERROR;
    }

	uint8_t myBuffer[2];

	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer

	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	*outputPointer = output;

	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t LSM6DS3CoreI2C::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
    if (!_i2c)
    {
        return IMU_GENERIC_ERROR;
    }

	status_t returnError = IMU_SUCCESS;
	uint8_t data[2];
	uint8_t dummy;

	I2C_Transaction i2cTransaction;

	i2cTransaction.slaveAddress = _address;

	data[0] = offset;
	data[1] = dataToWrite;

    i2cTransaction.writeBuf = data;
    i2cTransaction.writeCount = sizeof(data);
    i2cTransaction.readBuf = &dummy;
    i2cTransaction.readCount = 0;

    if (!I2C_transfer(_i2c, &i2cTransaction))
    {
        return IMU_GENERIC_ERROR;
    }

	return returnError;
}

status_t LSM6DS3CoreI2C::embeddedPage(void)
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80 );
	
	return returnError;
}

status_t LSM6DS3CoreI2C::basePage(void)
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00 );
	
	return returnError;
}
