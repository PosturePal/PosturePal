#ifndef __LSM6DS3_CORE_SPI_H__
#define __LSM6DS3_CORE_SPI_H__

#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <stdint.h>
#include "LSM6DS3_Core_Common.h"

//This is the core operational class of the driver.
//  LSM6DS3CoreSPI contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LSM6DS3 which inherits
//  this class.

class LSM6DS3CoreSPI
{
public:
	LSM6DS3CoreSPI(uint8_t);
	~LSM6DS3CoreSPI() { }
	
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
	uint8_t     chipSelectPin;
	SPI_Handle  spi;
};

#endif  // End of __LSM6DS3_CORE_SPI_H__ definition check
