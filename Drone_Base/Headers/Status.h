/*
* Status.h
*
* Created: 31.05.2018 13:20:51
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/


#ifndef STATUS_H_
#define STATUS_H_

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- STATUSCLASS DEFINES
//--------------------------------------------------------------------
#define STATUS_CLASS_SERIAL_BAUDRATE	115200UL 
#define TIME_INTERWAL_IN_MS				500UL			// for sensor init checking

//--------------------------------------------------------------------
//-- STATUSCLASS CONFIGS
//--------------------------------------------------------------------
#define STATUS_IS_USING_SENSOR			0x01	// 0x01 = USE, 0x00 = DON'T USE

#if( STATUS_IS_USING_SENSOR == 0x01)
#include "I2C_Sensor.h"
#endif

//--------------------------------------------------------------------
//-- STATUSCLASS CLASS
//--------------------------------------------------------------------
class StatusClass
{
	public:
	StatusClass(HardwareSerial& HardwareSerialObject, uint32_t baudrate, uint16_t delayInterval);
	
	void begin(void);
	void init(void);
	
	void printMessage(String message, String header = "\t");
	void printErrorMesage(String message);
	void printWarningMesage(String message);
	void printRegister8(uint8_t registerMask, uint8_t registerValue);
	void printRegister16(uint16_t registerMask, uint16_t registerValue);
	void printRegister8Value(uint8_t registerMask, float registerValue);
	
	HardwareSerial& getUARTObject();
	
#if( STATUS_IS_USING_SENSOR == 0x01)
	void checkStatus(I2C_Sensor &sensor);
#endif	
	
	private:
	HardwareSerial& _HardwareSerialObject;
	
	uint32_t _baudrate;
	
	uint16_t _delayInterval;
	
	void serialFlush(void);
};

extern StatusClass Status;

#endif /* STATUS_H_ */