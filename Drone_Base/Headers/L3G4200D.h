/*
* L3G4200D.h
*
* Created: 04.06.2018 11:21:59
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#ifndef L3G4200D_H_
#define L3G4200D_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- L3G4200D INCLUDES
//--------------------------------------------------------------------
#include "../Headers/I2C_Sensor.h"
#include "../Headers/I2C_Interface.h"

//--------------------------------------------------------------------
//-- L3G4200D PROGRAM DEFINES
//--------------------------------------------------------------------
#define L3G4200D_LOW_PASS_FILTER_ALPHA		0.3F
#define L3G4200D_X_OFFSET					0.15785F
#define L3G4200D_Y_OFFSET					-0.05804F
#define L3G4200D_Z_OFFSET					-0.02162F

//--------------------------------------------------------------------
//-- L3G4200D REGISTERS DEFINES
//--------------------------------------------------------------------
#define L3G4200D_REG_ID       				0x0F  // Should return 0xD3
#define L3G4200D_CTRL_REG1      			0x20
#define L3G4200D_CTRL_REG2      			0x21
#define L3G4200D_CTRL_REG3      			0x22
#define L3G4200D_CTRL_REG4      			0x23
#define L3G4200D_CTRL_REG5      			0x24
#define L3G4200D_REFERENCE      			0x25
#define L3G4200D_OUT_TEMP       			0x26
#define L3G4200D_STATUS_REG     			0x27
#define L3G4200D_OUT_X_L        			0x28
#define L3G4200D_OUT_X_H        			0x29
#define L3G4200D_OUT_Y_L        			0x2A
#define L3G4200D_OUT_Y_H        			0x2B
#define L3G4200D_OUT_Z_L        			0x2C
#define L3G4200D_OUT_Z_H        			0x2D
#define L3G4200D_FIFO_CTRL_REG  			0x2E
#define L3G4200D_FIFO_SRC_REG   			0x2F
#define L3G4200D_INT1_CFG       			0x30
#define L3G4200D_INT1_SRC       			0x31
#define L3G4200D_INT1_TSH_XH    			0x32
#define L3G4200D_INT1_TSH_XL    			0x33
#define L3G4200D_INT1_TSH_YH    			0x34
#define L3G4200D_INT1_TSH_YL    			0x35
#define L3G4200D_INT1_TSH_ZH    			0x36
#define L3G4200D_INT1_TSH_ZL				0x37
#define L3G4200D_INT1_DURATION				0x38

//--------------------------------------------------------------------
//-- L3G4200D REGISTERS SPECIFIC DEFINES
//--------------------------------------------------------------------
//L3G4200D REGISTER -> CTRL_REG1 - SPECIFIC MODES:
#define L3G4200D_ENABLE_ALL_AXES			0x07
#define L3G4200D_ENABLE_NORMAL_POWER_MODE	0x01
#define L3G4200D_DTBW_MASK					0xF0
#define L3G4200D_SCALE_MASK					0x30

//L3G4200D REGISTER -> CTRL_REG3 - SPECIFIC MODES:
#define L3G4200D_ENABLE_DR_INTERRUPT		0x08

//L3G4200D REGISTER -> CTRL_REG4 - SPECIFIC MODES:
#define L3G4200D_ENABLE_BDU					0x80

//L3G4200D REGISTER -> DATA
#define NUMBER_OF_REGISTERS					0x06
#define L3G4200D_ENABLE_AUTOINCREMENT		0x80
#define X_LOW_BYTE							0x00
#define X_HIGH_BYTE							0x01
#define Y_LOW_BYTE							0x02
#define Y_HIGH_BYTE							0x03
#define Z_LOW_BYTE							0x04
#define Z_HIGH_BYTE							0x05

//--------------------------------------------------------------------
//-- L3G4200D OTHER DEFINES
//--------------------------------------------------------------------
#define L3G4200D_CLEAR_REGISTER				0x00

//--------------------------------------------------------------------
//-- L3G4200D ENUMS
//--------------------------------------------------------------------
enum eL3G4200D_Scale
{
	SCALE_250DPS = 0,
	SCALE_500DPS,
	SCALE_2000DPSa,
	SCALE_2000DPSb
};

enum eL3G4200D_DatarateAndBandwidth
{
	DRBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
	DRBW_100_25a,
	DRBW_100_25b,
	DRBW_100_25c,
	DRBW_200_125,
	DRBW_200_25,
	DRBW_200_50,
	DRBW_200_70,
	DRBW_400_20,
	DRBW_400_25,
	DRBW_400_50,
	DRBW_400_110,
	DRBW_800_30,
	DRBW_800_35,
	DRBW_800_50,
	DRBW_800_110  // 800 Hz ODR, 110 Hz bandwidth
};

#ifndef ENABLE_LOW_PASS
#define ENABLE_LOW_PASS
enum eSetting
{
	DISABLE = 0,
	ENABLE = 1
};
#endif /* ENABLE_LOW_PASS */

#ifndef AXESSTRUCTURE
#define AXESSTRUCTURE
struct sIntAxes
{
	int16_t X;
	int16_t Y;
	int16_t Z;
};

struct sFloatAxes
{
	float X;
	float Y;
	float Z;
};
#endif /* AXESSTRUCTURE */

//--------------------------------------------------------------------
//-- L3G4200D REGISTERS SPECIFIC DEFINES
//--------------------------------------------------------------------
#define L3G4200D_ADDRESS					0x69

//--------------------------------------------------------------------
//-- L3G4200D CLASS
//--------------------------------------------------------------------
class L3G4200D : public I2C_Sensor, public I2C_Interface
{
public:
	L3G4200D(String sensorName, uint8_t I2CAddress);
	
	eBeginStatus begin(void);
	void init(void);
	void read(void);
	String getDeviceName();
	uint8_t getDeviceAddress();
	
	eL3G4200D_DatarateAndBandwidth getDataRateAndBandwidthFromDevice(void);
	void setDataRateAndBandwidth(eL3G4200D_DatarateAndBandwidth value);
	eL3G4200D_Scale getScaleFromDevice(void);
	void setScale(eL3G4200D_Scale scale);
		
	sIntAxes getRaw(void);
	sFloatAxes getFiltered(void);
	sFloatAxes getScaled(void);
	sFloatAxes getTemporaryTraveledAngle(void);
	
	void readTemperature(void);
	uint8_t getTemperature(void);
	
	void lowPassFilterSetting(eSetting flag);
	
	void calibrationSetting(eSetting flag);
	void performCalibrationProcedure(void);
	
private:
	String _sensorName;
	uint8_t _I2CAddress;
	eL3G4200D_DatarateAndBandwidth _datarateAndBandwidth;
	eL3G4200D_Scale _scale;
	float _resolution;
	uint8_t _temperature;
	
	sIntAxes _rawAngularVelocities;
	sFloatAxes _filteredAngularVelocities;
	sFloatAxes _angularScaledVelocities;
	sFloatAxes temporaryAngle;
	
	eSetting _lowPassEnableFlag;
	eSetting _calibrationEnableFlag;
	
	uint8_t getDeviceID(void);
	
	void lowPassFilter(float alpha);
	void calculateTemporaryTraveledAngle();
	
	void setCTRL_REG1(void);
	void setCTRL_REG2(void);
	void setCTRL_REG3(void);
	void setCTRL_REG4(void);
	void setCTRL_REG5(void);
};

//--------------------------------------------------------------------
//-- L3G4200D EXTERN OBJECT
//--------------------------------------------------------------------
extern L3G4200D Gyroscope;

#endif /* L3G4200D_H_ */