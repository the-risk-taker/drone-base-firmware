/*
* I2C_Sensor.h
*
* Created: 01.06.2018 13:00:51
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#ifndef _SENSOR_h
#define _SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- SENSOR ENUMS
//--------------------------------------------------------------------
typedef enum
{
	BEGIN_ERROR = 0x00,
	BEGIN_COMPLETED = 0x01
}eBeginStatus;

//--------------------------------------------------------------------
//-- SENSOR
//--------------------------------------------------------------------
class I2C_Sensor
{
	public:
	virtual eBeginStatus begin(void) = 0;
	virtual void init(void) = 0;
	
	virtual String getDeviceName() = 0;
	virtual uint8_t getDeviceAddress() = 0;
	virtual uint8_t getDeviceID() = 0;
	
	virtual void read(void) = 0;
};

#endif