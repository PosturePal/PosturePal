#ifndef __LSM6DS3_CORE_I2C_H__
#define __LSM6DS3_CORE_I2C_H__

#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <stdint.h>
#include "LSM6DS3_Core_Common.h"

#define LSM6DS3_ADDRESS_LOW     (0b1101010)
#define LSM6DS3_ADDRESS_HIGH    (0b1101011)

//This is the core operational class of the driver.
//  LSM6DS3CoreI2C contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LSM6DS3 which inherits
//  this class.

class LSM6DS3CoreI2C
{
public:
	LSM6DS3CoreI2C(uint8_t address = LSM6DS3_ADDRESS_LOW);
	~LSM6DS3CoreI2C() { }
	
	status_t beginCore(void);
	status_t endCore(void);
	
	//The following utilities read and write to the IMU

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t);
	
	//readRegister reads one 8-bit register
	status_t readRegister(uint8_t *, uint8_t);
	
	//Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//  Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t*, uint8_t offset);
	
	//Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);
	
	//Change to embedded page
	status_t embeddedPage(void);
	
	//Change to base page
	status_t basePage(void);
	
private:
	I2C_Handle  _i2c;
	uint8_t _address;
};

#endif  // End of __LSM6DS3_CORE_I2C_H__ definition check
