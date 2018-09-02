/* 
* HMC5883L.h
*
* Created: 12.06.2018 11:19:51
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#ifndef __HMC5883L_H__
#define __HMC5883L_H__

//--------------------------------------------------------------------
//-- HMC5883L INCLUDES
//--------------------------------------------------------------------
#include "../Headers/I2C_Sensor.h"
#include "../Headers/I2C_Interface.h"

//--------------------------------------------------------------------
//-- HMC5883L PROGRAM DEFINES
//--------------------------------------------------------------------
#define HMC5883L_ENABLE_MORE_INSTANCES		0x00
#define CRACOW_DECLINATION_ANGLE			5.18F	// For 20.06.2018 Magnetic declination: +5° 18' 

//--------------------------------------------------------------------
//-- HMC5883L DEBUG DEFINES
//--------------------------------------------------------------------
#define HMC5883L_CALIBRATION_DEBUG			0x00 // 0x00=DISABLE 0x01=ENABLE
//--------------------------------------------------------------------
//-- HMC5883L REGISTERS DEFINES
//--------------------------------------------------------------------
#define HMC5883L_CONFIGURATION_A			0x00
#define HMC5883L_CONFIGURATION_B			0x01
#define HMC5883L_MODE						0x02
#define HMC5883L_OUT_X_MSB					0x03
#define HMC5883L_OUT_X_LSB					0x04
#define HMC5883L_OUT_Z_MSB					0x05
#define HMC5883L_OUT_Z_LSB					0x06
#define HMC5883L_OUT_Y_MSB					0x07
#define HMC5883L_OUT_Y_LSB					0x08
#define HMC5883L_STATUS						0x09
#define HMC5883L_IDENTIFICATION_A			0x0A  // should return 0x48
#define HMC5883L_IDENTIFICATION_B			0x0B  // should return 0x34
#define HMC5883L_IDENTIFICATION_C			0x0C  // should return 0x33

//--------------------------------------------------------------------
//-- HMC5883L REGISTERS SPECIFIC DEFINES
//--------------------------------------------------------------------
#define HMC5883L_ADDRESS         			0x1E
#define HMC5883L_CLEAR_REGISTER				0x00

//HMC5883L REGISTER -> DATA
#define NUMBER_OF_REGISTERS					0x06
#define HMC5883L_X_HIGH_BYTE				0x00
#define HMC5883L_X_LOW_BYTE					0x01
#define HMC5883L_Z_HIGH_BYTE				0x02
#define HMC5883L_Z_LOW_BYTE					0x03
#define HMC5883L_Y_HIGH_BYTE				0x04
#define HMC5883L_Y_LOW_BYTE					0x05

#define HMC5883L_LOW_LIMIT					(243.0F/390.0F)	// 243 when gain = 5
#define HMC5883L_HIGH_LIMIT					(575.0F/390.0F) // 575 when gain = 5

//HMC5883L REGISTER -> MODE
#define ENABLE_HIGH_SPEED_MODE				0x80

//HMC5883L SELF TEST
#define HMC5883L_SF_X_PARAMETER				1.16F
#define HMC5883L_SF_Y_PARAMETER				(HMC5883L_SF_X_PARAMETER)
#define HMC5883L_SF_Z_PARAMETER				1.08F

//--------------------------------------------------------------------
//-- HMC5883L ENUMS
//--------------------------------------------------------------------
enum eHMC5883LAverageSamples
{
	SAMPLES_1 = 0b00,
	SAMPLES_2,
	SAMPLES_4,
	SAMPLES_8
};

enum eHMC5883LOutputDataRate 
{
	ODR_0_75_HZ = 0b000,
	ODR_1_5_HZ,
	ODR_3_0_HZ,
	ODR_7_5_HZ,
	ODR_15_HZ,
	ODR_30_HZ,
	ODR_75_HZ
};

enum eHMC5883LMeasurementMode
{
	NORMAL_MEASUREMENT = 0b00,
	POSITIVE_BIAS,
	NEGATIVIE_BIAS
};

enum eHMC5883LResolution
{
	//Actually, gain is chosen here.
	RESOLUTION_0_73 = 0b00, // 0.73mG/LSB
	RESOLUTION_0_92,
	RESOLUTION_1_22,
	RESOLUTION_1_52,
	RESOLUTION_2_27,
	RESOLUTION_2_56,
	RESOLUTION_3_03,
	RESOLUTION_4_35			// 4.35mG/LSB
};

enum eHMC5883LOperationMode
{
	CONTINUOUS_MEASUREMENT_MODE = 0b00,
	SINGLE_MEASUREMENT_MODE,
	IDLE_MEASUREMENT_MODE,
};

enum eHMC5883LBitStatus
{
	SET = 0,
	CLEAR = 1
};

enum eHMC5883LOperationStatus
{
	DISABLED = 0,
	ENABLED = 1
};

enum eHMC5883LSelfTest
{
	FAILED = 0,
	PASSED = 1
};

//--------------------------------------------------------------------
//-- HMC5883L STRUCTURES
//--------------------------------------------------------------------
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
//-- HMC5883L CLASS
//--------------------------------------------------------------------
class HMC5883L : public I2C_Sensor, public I2C_Interface
{
public:
	HMC5883L(String sensorName, uint8_t I2CAddress, uint8_t dataReadyPin);
	
	eBeginStatus begin(void) override;
	void init(void) override;
	void read(void) override;
	
	String getDeviceName(void) override;
	uint8_t getDeviceAddress(void) override;
	uint8_t getDeviceID(void) override;	

	bool getDataReadyFlag(uint8_t timeout = 11);
	
	sIntAxes getRaw(void);
	sFloatAxes getScaled(void);
	
private:
	const String _sensorName;
	const uint8_t _I2CAddress;
	const uint8_t _dataReadyPin;
	
	volatile bool dataReadyFlag;
	
	eHMC5883LOperationStatus singleModeStatus;
	eHMC5883LSelfTest selftTestStatus;
	
	float _resolution;
	
	sIntAxes rawData;
	sFloatAxes gainFactor;
	sFloatAxes offsetFactor;
	sFloatAxes scaledData;
	
	void setResolution(eHMC5883LResolution resolution);
	uint16_t getGainLSBperGauss(eHMC5883LResolution resolution);
	
	void setExternalInterrupts(void);
	
	void setConfigurationRegisterA(	eHMC5883LMeasurementMode measurementMode, 
									eHMC5883LOutputDataRate outputDataRate,
									eHMC5883LAverageSamples samplesToAverage);
	void setConfigurationRegisterB(	eHMC5883LResolution resolution);
	void setModeRegister(eHMC5883LOperationMode operationMode);
	
	void performSelfTestCalibration(void);
	void useSelfTestCompensationParameters(void);
	void useHardAndSoftIronCompensationParameters(void);
	
	uint8_t readStatusRegister(void);
	eHMC5883LBitStatus checkReadyBit(void);
	eHMC5883LBitStatus checkLockBit(void);

	// Static ISR methods
	static void isrA(void);
#if (HMC5883L_ENABLE_MORE_INSTANCES == 0x01)
	static void isrB(void);
#endif
	
	// Class member ISR handler
	void handleInterrupt(void);
	
	static HMC5883L * instanceA;
#if (HMC5883L_ENABLE_MORE_INSTANCES == 0x01)
	static HMC5883L * instanceB;
#endif
	
};

//--------------------------------------------------------------------
//-- HMC5883L EXTERN OBJECT
//--------------------------------------------------------------------
extern HMC5883L Magnetometer;
#endif //__HMC5883L_H__
