/* 
* BMP085.h
*
* Created: 06.06.2018 21:21:22
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#ifndef __BMP085_H__
#define __BMP085_H__

//--------------------------------------------------------------------
//-- BMP085 INCLUDES
//--------------------------------------------------------------------
#include "../Headers/I2C_Sensor.h"
#include "../Headers/I2C_Interface.h"

//--------------------------------------------------------------------
//-- BMP085 PROGRAM DEFINES
//--------------------------------------------------------------------
#define BMP085_SEA_LEVEL_PREASSURE			101325
#define ENABLE_MOVING_AVERAGE_FILTERING		0x01	// 0x01 = Enable; Other = Disable
#define BMP085_MOVING_AVERAGE_RANGE			20
//--------------------------------------------------------------------
//-- BMP085 REGISTERS DEFINES
//--------------------------------------------------------------------
#define BMP085_AC1_MSB						0xAA
#define BMP085_AC1_LSB						0xAB
#define BMP085_AC2_MSB						0xAC
#define BMP085_AC2_LSB						0xAD
#define BMP085_AC3_MSB						0xAE
#define BMP085_AC3_LSB						0xAF
#define BMP085_AC4_MSB						0xB0
#define BMP085_AC4_LSB						0xB1
#define BMP085_AC5_MSB						0xB2
#define BMP085_AC5_LSB						0xB3
#define BMP085_AC6_MSB						0xB4
#define BMP085_AC6_LSB						0xB5
#define BMP085_B1_MSB						0xB6
#define BMP085_B1_LSB						0xB7
#define BMP085_B2_MSB						0xB8
#define BMP085_B2_LSB						0xB9
#define BMP085_MB_MSB						0xBA
#define BMP085_MB_LSB						0xBB
#define BMP085_MC_MSB						0xBC
#define BMP085_MC_LSB						0xBD
#define BMP085_MD_MSB						0xBE
#define BMP085_MD_LSB						0xBF

#define BMP085_ADDRESS						0x77
#define BMP085_REG_ID						0xD0 // Should return 0x55

#define BMP085_REG_CONTROL					0xF4
#define BMP085_REG_GET_TEMP					0x2E
#define BMP085_TEMPERATURE_MSB				0xF6
#define BMP085_TEMPERATURE_LSB				0xF7

#define BMP085_REG_GET_PRESSURE				0x34
#define BMP085_PRESSURE_MSB					0xF6
#define BMP085_PRESSURE_LSB					0xF7
#define BMP085_PRESSURE_XLSB				0xF8

//--------------------------------------------------------------------
//-- BMP085 ENUMS
//--------------------------------------------------------------------
enum eBMP085_Mode
{							// Conversion time [ms]:
	ULTRA_LOW_POWER = 0x00,	// 4.5
	STANDARD,				// 7.5
	HIGH_RESOLUTION,		// 13.5
	ULTRA_HIGH_RESOLUTION	// 25.5
};

//--------------------------------------------------------------------
//-- BMP085 CLASS
//--------------------------------------------------------------------

class BMP085 : public I2C_Sensor, public I2C_Interface
{
public:
	BMP085(String sensorName, uint8_t I2CAddress);

	eBeginStatus begin(void);
	void init(void);
	
	uint8_t getDeviceAddress(void);
	String getDeviceName();

	void setMode(eBMP085_Mode mode);
	void read(void);
		
	float getTemperature(void);
	int32_t getPressure(void);
	float getAltitude(void);

private:
	const String _sensorName;
	const uint8_t _I2CAddress;
	
	// Calibration Values
	int16_t		BMP085_AC1,
				BMP085_AC2,
				BMP085_AC3;
	uint16_t	BMP085_AC4,
				BMP085_AC5,
				BMP085_AC6;
	int16_t		BMP085_B1,
				BMP085_B2,
				BMP085_MB,
				BMP085_MC,
				BMP085_MD;
	
	int32_t		BMP085_B5,
				BMP085_UT,
				BMP085_UP;
	
	// Data in SI units
	float		temperature,
				altitude;
	
	int32_t		pressure;
	
	eBMP085_Mode _mode;
	
	uint8_t getDeviceID(void);
	
	uint8_t getTimeInterval(void);
	
	bool readCalibrationCoefficients(void);
	bool readUncompensatedTemperature(void);
	void calculateTemperature(void);
	
	bool readUncompensatedPressure(void);
	void calculatePressure(void);
	void calculateAltitude(uint32_t seaLevelPressure = BMP085_SEA_LEVEL_PREASSURE);
	
	uint16_t read16(uint8_t regMSB, uint8_t regLSB);
	
#if( ENABLE_MOVING_AVERAGE_FILTERING == 0x01)
	int32_t movingAverage(int32_t newValue);
	void moveElements(int32_t table[], uint8_t lastElementIndex);
#endif
};

//--------------------------------------------------------------------
//-- BMP085 EXTERN OBJECT
//--------------------------------------------------------------------
extern BMP085 Barometer;

#endif //__BMP085_H__
