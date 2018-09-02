/*
* Status.cpp
*
* Created: 31.05.2018 14:18:50
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

//--------------------------------------------------------------------
//-- STATUSCLASS INCLUDES
//--------------------------------------------------------------------
#include "../Headers/Status.h"

//--------------------------------------------------------------------
//-- STATUSCLASS CLASS
//--------------------------------------------------------------------
StatusClass::StatusClass(HardwareSerial& HardwareSerialObject, uint32_t baudrate, uint16_t delayInterval) : _HardwareSerialObject(HardwareSerialObject), _baudrate(baudrate), _delayInterval(delayInterval)
{
};

void StatusClass::begin(void)
{
	this->serialFlush();
	this->init();
}

void StatusClass::init(void)
{
	this->printMessage("Serial initialized with baudrate = " + String(this->_baudrate));
}

void StatusClass::serialFlush()
{
	this->_HardwareSerialObject.begin(this->_baudrate);

	while(this->_HardwareSerialObject.available() > 0)
	{
		this->_HardwareSerialObject.read();
	}
}

void StatusClass::printMessage(String message, String header)
{
	this->_HardwareSerialObject.println(String(millis()) + header + message);
}

void StatusClass::printErrorMesage(String message)
{
	this->printMessage(message, "\tERROR\t");
}

void StatusClass::printWarningMesage(String message)
{
	this->printMessage(message, "\tWARNING\t");
}

void StatusClass::printRegister8(uint8_t registerMask, uint8_t registerValue)
{
	String bytes = "Register 0x" + String(registerMask, HEX) + "\tRegister Value = " + String(registerValue, HEX) + " (HEX)\n";
	
	bytes += "\t| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |\n";
	bytes += "\t| ";
	for(uint8_t i = 0; i < (8*sizeof(registerMask)); i++)
	{
		bytes += String((registerValue >> ((8*sizeof(registerMask) - 1)-i)) & 1);
		bytes += " | ";
	}
	
	this->printMessage(bytes);
}

void StatusClass::printRegister16(uint16_t registerMask, uint16_t registerValue)
{
	String bytes = "Register 0x" + String(registerMask, HEX) + "\tRegister Value = " + String(registerValue, HEX) + " (HEX)\n";
	
	bytes += "\t| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |\n";
	bytes += "\t| ";
	for(uint8_t i = 0; i < (8*sizeof(registerMask)); i++)
	{
		bytes += String((registerValue >> ((8*sizeof(registerMask) - 1)-i)) & 1);
		bytes += " | ";
	}
	
	this->printMessage(bytes);
}

void StatusClass::printRegister8Value(uint8_t registerMask, float registerValue)
{
	this->printMessage("Register 0x" + String(registerMask, HEX) + " = " + registerValue + " (DEC)");
}

HardwareSerial& StatusClass::getUARTObject()
{
	return this->_HardwareSerialObject;
}

#if( STATUS_IS_USING_SENSOR == 0x01)
void StatusClass::checkStatus(I2C_Sensor &sensor)
{
	if(BEGIN_ERROR == sensor.begin())
	{
		while(1)
		{
			this->printMessage("Not found " + sensor.getDeviceName() + " searching device with ID = 0x" + String(sensor.getDeviceAddress(), HEX));
			delay(this->_delayInterval);
		}
	}
	else
	{
		this->printMessage("Found " + sensor.getDeviceName() + " searching device with ID = 0x" + String(sensor.getDeviceAddress(), HEX));
	}
}
#endif

//--------------------------------------------------------------------
//-- STATUSCLASS EXTERN OBJECT
//--------------------------------------------------------------------
StatusClass Status(Serial, STATUS_CLASS_SERIAL_BAUDRATE, TIME_INTERWAL_IN_MS);