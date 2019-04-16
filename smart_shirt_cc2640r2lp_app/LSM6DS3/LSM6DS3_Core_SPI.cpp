#include <LSM6DS3.h>
#include "stdint.h"
#include "delay.h"
#include "board.h"
#include "LSM6DS3_Core_SPI.h"

//****************************************************************************//
//
//  LSM6DS3CoreSPI functions.
//
//  Construction arguments:
//  uint8_t cs: chip select pin number
//
//
//****************************************************************************//
LSM6DS3CoreSPI::LSM6DS3CoreSPI(uint8_t cs) :
    chipSelectPin(cs)
{
}

status_t LSM6DS3CoreSPI::beginCore(void)
{
    if (spi == NULL)
    {
        // Initialize the SPI driver.
        SPI_init();

        SPI_Params spiParams;

        // Create a SPI with data processing off.
        SPI_Params_init(&spiParams);

        // Blocking or Callback mode
        spiParams.transferMode = SPI_MODE_BLOCKING;

        // Transfer timeout in system ticks (Not supported with all implementations)
        spiParams.transferTimeout = SPI_WAIT_FOREVER;

        // Callback function pointer
        spiParams.transferCallbackFxn = NULL;

        // Master or Slave mode
        spiParams.mode = SPI_MASTER;

        // SPI bit rate in Hz
        spiParams.bitRate = 4000000;

        // SPI data frame size in bits
        spiParams.dataSize = 8;

        // SPI frame format
        spiParams.frameFormat = SPI_POL0_PHA0;

        spi = SPI_open(CC2640R2_LAUNCHXL_SPI0, &spiParams);

        if (spi == NULL)
        {
            // SPI_open() failed
            return IMU_HW_ERROR;
        }
    }

    // Data is captured on rising edge of clock (CPHA = 0)
    // Base value of the clock is HIGH (CPOL = 1)

    // initalize the  data ready and chip select pins:

    GPIO_write(chipSelectPin, BOARD_GPIO_HIGH);

    //Spin for a few ms
    delay(100);

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
//  LSM6DS3CoreSPI functions.
//
//  Construction arguments:
//
//
//****************************************************************************//
status_t LSM6DS3CoreSPI::endCore(void)
{
    GPIO_write(chipSelectPin, BOARD_GPIO_HIGH);

    SPI_close(spi);

    spi = NULL;

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
status_t LSM6DS3CoreSPI::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
    if (!spi)
    {
        return IMU_GENERIC_ERROR;
    }

    // take the chip select low to select the device:
    GPIO_write(chipSelectPin, BOARD_GPIO_LOW);
    //delay(1);

    SPI_Transaction transaction;
    uint8_t rxByte;
    uint8_t txByte;

    transaction.count = 1;
    transaction.rxBuf = (void *) &rxByte;
    transaction.txBuf = (void *) &txByte;
    transaction.arg = NULL;

    txByte = offset | 0x80;

    // send the device the register you want to read:
    if (!SPI_transfer(spi, &transaction))
    {
        return IMU_GENERIC_ERROR;
    }

    txByte = 0xFF;

    for (size_t i = 0; i < length; i++)
    {
        if (SPI_transfer(spi, &transaction))
        {
            outputPointer[i] = rxByte;
        }
        else
        {
            return IMU_GENERIC_ERROR;
        }
    }

    // take the chip select high to de-select:
    GPIO_write(chipSelectPin, BOARD_GPIO_HIGH);
    //delay(1);

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
status_t LSM6DS3CoreSPI::readRegister(uint8_t* outputPointer, uint8_t offset)
{
	//Return value
	uint8_t result;
	status_t returnError = IMU_SUCCESS;

    // take the chip select low to select the device:
    GPIO_write(chipSelectPin, BOARD_GPIO_LOW);
    //delay(1);

    SPI_Transaction transaction;
    uint8_t rxByte;
    uint8_t txByte;

    transaction.count = 1;
    transaction.rxBuf = (void *) &rxByte;
    transaction.txBuf = (void *) &txByte;
    transaction.arg = NULL;

    txByte = offset | 0x80;

    if (!SPI_transfer(spi, &transaction))
    {
        return IMU_GENERIC_ERROR;
    }

    txByte = 0xFF;

    if (!SPI_transfer(spi, &transaction))
    {
        return IMU_GENERIC_ERROR;
    }

    result = rxByte;

    // take the chip select high to de-select:
    GPIO_write(chipSelectPin, BOARD_GPIO_HIGH);
    //delay(1);

    returnError = IMU_SUCCESS;

    if (result == 0xFF)
    {
        //we've recieved all ones, report
        returnError = IMU_ALL_ONES_WARNING;
    }

	outputPointer[0] = result;

	return returnError;
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
status_t LSM6DS3CoreSPI::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
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
status_t LSM6DS3CoreSPI::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	status_t returnError = IMU_SUCCESS;

    // take the chip select low to select the device:
    GPIO_write(chipSelectPin, BOARD_GPIO_LOW);
    //delay(1);

    SPI_Transaction transaction;
    uint8_t rxByte;
    uint8_t txByte;

    transaction.count = 1;
    transaction.rxBuf = (void *) &rxByte;
    transaction.txBuf = (void *) &txByte;
    transaction.arg = NULL;

    txByte = offset;

    if (!SPI_transfer(spi, &transaction))
    {
        return IMU_GENERIC_ERROR;
    }

    txByte = dataToWrite;

    if (!SPI_transfer(spi, &transaction))
    {
        return IMU_GENERIC_ERROR;
    }

    // decrement the number of bytes left to read:
    // take the chip select high to de-select:
    GPIO_write(chipSelectPin, BOARD_GPIO_HIGH);
    //delay(1);

	return returnError;
}

status_t LSM6DS3CoreSPI::embeddedPage(void)
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80 );
	
	return returnError;
}

status_t LSM6DS3CoreSPI::basePage(void)
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00 );
	
	return returnError;
}
