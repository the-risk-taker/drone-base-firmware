/*
* I2C_Interface.h
*
* Created: 31.05.2018 14:18:50
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#ifndef _I2CINTERFACE_h
#define _I2CINTERFACE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Wire.h"

#ifndef I2C_TRANSMISSION_STATUS
#define I2C_TRANSMISSION_STATUS
	typedef enum eI2CStatus
	{
		SUCCESS = 0,
		DATA_TOO_LONG_TO_FIT_IN_TRANSMIT_BUFFER = 1,
		RECEIVED_NACK_ON_TRANSMIT_OF_ADDRESS = 2,
		RECEIVED_NACK_ON_TRANSMIT_OF_DATA = 3,
		OTHER_ERROR = 4
	}eI2CStatus;
#endif

class I2C_Interface
{
protected:
	eI2CStatus writeCommand(uint8_t slaveAddress, uint8_t command);
	eI2CStatus writeByte(uint8_t slaveAddress, uint8_t slaveRegisterAddress, uint8_t data);
	uint8_t readByte(uint8_t slaveAddress, uint8_t slaveRegisterAddress);
	eI2CStatus readByte(uint8_t slaveAddress, uint8_t slaveRegisterAddress, uint8_t &destination);
	eI2CStatus readBytes(uint8_t slaveAddress, uint8_t slaveRegisterAddress, uint8_t numbersOfBytesToRead, uint8_t *destination);
};

#endif