/*
* ADXL345.h
*
* Created: 03.06.2018 21:31:22
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

//--------------------------------------------------------------------
//-- ADXL345 REGISTERS MAP:
//-- https://www.i2cdevlib.com/devices/adxl345#registers
//--------------------------------------------------------------------

#ifndef _ADXL345_h
#define _ADXL345_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- ADXL345 INCLUDES
//--------------------------------------------------------------------
#include "../Headers/I2C_Sensor.h"
#include "../Headers/I2C_Interface.h"

//--------------------------------------------------------------------
//-- ADXL345 PROGRAM DEFINES
//--------------------------------------------------------------------
#define NUMBER_OF_INIT_RETRIES				10
#define ADXL345_LOW_PASS_FILTER_ALPHA		0.3F

//--------------------------------------------------------------------
//-- ADXL345 DEBUG DEFINES
//--------------------------------------------------------------------

#define ADXL345_DEBUG						0x00	// 0x01 = Enable; Other = Disable
#define ADXL345_ENABLE_CALIBRATION_PROCESS	0x00	// 0x01 = Enable; Other = Disable

//--------------------------------------------------------------------
//-- ADXL345 REGISTERS DEFINES
//--------------------------------------------------------------------
#define ADXL345_REG_ID						0x00   // Should return 0xE5
#define ADXL345_THRESH_TAP					0x1D   // Tap threshold
#define ADXL345_OFSX						0x1E   // X-axis offset
#define ADXL345_OFSY						0x1F   // Y-axis offset
#define ADXL345_OFSZ						0x20   // Z-axis offset
#define ADXL345_DUR							0x21   // Tap duration
#define ADXL345_LATENT						0x22   // Tap latency
#define ADXL345_WINDOW						0x23   // Tap window
#define ADXL345_THRESH_ACT					0x24   // Activity threshold
#define ADXL345_THRESH_INACT				0x25   // Inactivity threshold
#define ADXL345_TIME_INACT					0x26   // Inactivity time
#define ADXL345_ACT_INACT_CTL				0x27   // Axis enable control for activity/inactivity detection
#define ADXL345_THRESH_FF					0x28   // Free-fall threshold
#define ADXL345_TIME_FF						0x29   // Free-fall time
#define ADXL345_TAP_AXES					0x2A   // Axis control for single/double tap
#define ADXL345_ACT_TAP_STATUS				0x2B   // Source of single/double tap
#define ADXL345_BW_RATE						0x2C   // Data rate and power mode control
#define ADXL345_POWER_CTL					0x2D   // Power-saving features control
#define ADXL345_INT_ENABLE					0x2E   // Interrupt enable control
#define ADXL345_INT_MAP						0x2F   // Interrupt mapping control
#define ADXL345_INT_SOURCE					0x30   // Source of interrupts
#define ADXL345_DATA_FORMAT     			0x31   // Data format control
#define ADXL345_DATAX0       				0x32   // X-axis data 0
#define ADXL345_DATAX1       				0x33   // X-axis data 1
#define ADXL345_DATAY0       				0x34   // Y-axis data 0
#define ADXL345_DATAY1       				0x35   // Y-axis data 1
#define ADXL345_DATAZ0        				0x36   // Z-axis data 0
#define ADXL345_DATAZ1         				0x37   // Z-axis data 1
#define ADXL345_FIFO_CTL        			0x38   // FIFO control
#define ADXL345_FIFO_STATUS     			0x39   // FIFO status

//--------------------------------------------------------------------
//-- ADXL345 REGISTERS SPECIFIC DEFINES
//--------------------------------------------------------------------
//ADXL345 I2C ADDRESS
#define ADXL345_ADDRESS         			0x53

//ADXL345 REGISTER -> POWER_CTL - SPECIFIC MODES:
#define STANDBY_MODE						0x00
#define MEASURE_MODE						0x08

//ADXL345 REGISTER -> DATA_FORMAT - SPECIFIC MODES:
#define JUSTIFY_LEFT						0x04
#define JUSTIFY_RIGHT						0x00
#define FULL_RESOLUTION_MODE				0x08

//ADXL345 REGISTER -> FIFO_CTL - SPECIFIC MODES:
#define BAYPASS_FIFO						0x00

//ADXL345 REGISTER -> BW_RATE - SPECIFIC MODES:
#define	DATA_RATE_MASK						0x0F
#define	RANGE_MASK							0x03
#define REDUCED_POWER						0x10
#define NORMAL_POWER						0x00

//ADXL345 REGISTER -> DATA
#define NUMBER_OF_REGISTERS					0x06
#define X_LOW_BYTE							0x00
#define X_HIGH_BYTE							0x01
#define Y_LOW_BYTE							0x02
#define Y_HIGH_BYTE							0x03
#define Z_LOW_BYTE							0x04
#define Z_HIGH_BYTE							0x05

//ADXL345 REGISTER -> THRESH_FF - SPECIFIC MODES:
#define FF_THRESHOLD_SCALE_FACTOR			0.0625f
#define FF_THRESHOLD_MIN_VALUE				0x05
#define FF_THRESHOLD_MAX_VALUE				0x09
#define FF_TIME_SCALE_FACTOR				5.0f
#define FF_TIME_MIN_VALUE					0x14
#define FF_TIME_MAX_VALUE					0x46

//ADXL345 REGISTER -> THRESH_TAP - SPECIFIC MODES:
#define TAP_THRESHOLD_SCALE_FACTOR			0.0625f
#define TAP_THRESHOLD_MIN_VALUE				0x10
#define TAP_THRESHOLD_MAX_VALUE				0xFF

//ADXL345 REGISTER -> TAP_AXES - SPECIFIC MODES:
#define ENABLE_TAP_ON_X						0x04
#define ENABLE_TAP_ON_Y						0x02
#define ENABLE_TAP_ON_Z						0x01
#define ENABLE_SUPPRES_FOR_DT				0x08

//ADXL345 REGISTER -> DUR - SPECIFIC MODES:
#define TAP_DUR_SCALE_FACTOR				0.6250f //[ms/LSB]
#define TAP_DUR_MIN_VALUE					0x00
#define TAP_DUR_MAX_VALUE					0xFF

//ADXL345 REGISTER -> WINDOW - SPECIFIC MODES:
#define DOUBLE_TAP_WINDOW_FACTOR			1.250f //[ms/LSB]
#define DOUBLE_TAP_WINDOW_MIN_VALUE			0x00
#define DOUBLE_TAP_WINDOW_MAX_VALUE			0xFF

//ADXL345 REGISTER -> LATENT - SPECIFIC MODES:
#define DOUBLE_TAP_LATENT_FACTOR			1.250f //[ms/LSB]
#define DOUBLE_TAP_LATENT_MIN_VALUE			0x00
#define DOUBLE_TAP_LATENT_MAX_VALUE			0xFF

//ADXL345 REGISTER -> INT_MAP - SPECIFIC MODES:
#define ALL_EVENTS_ON_INT_1_PIN				0x00
#define ALL_EVENTS_ON_INT_2_PIN				0xFF

//ADXL345 REGISTER -> INT_ENABLE - SPECIFIC MODES:
#define ENABLE_FF_INT_MASK					0x04
#define ENABLE_DT_INT_MASK					0x20
#define ENABLE_ST_INT_MASK					0x40

//ADXL345 REGISTER -> INT_MAP - SPECIFIC MODES:
#define FF_INTERRUPT_MASK					0x04
#define DT_INTERRUPT_MASK					0x20
#define ST_INTERRUPT_MASK					0x40

//--------------------------------------------------------------------
//-- ADXL345 OTHER DEFINES
//--------------------------------------------------------------------
#define GRAVITY_ON_EARTH					9.80665F   // Earth's gravity in m/s^2
#define ADXL345_CLEAR_REGISTER				0x00

//--------------------------------------------------------------------
//-- ADXL345 ENUMS
//--------------------------------------------------------------------
enum eADXL345_Range
{
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum eADXL345_DatarateBandwidth
{
	ARTBW_010_005 = 0, // 0.1 Hz ODR, 0.05Hz bandwidth
	ARTBW_020_010,
	ARTBW_039_020,
	ARTBW_078_039,
	ARTBW_156_078,
	ARTBW_313_156,
	ARTBW_125_625,
	ARTBW_25_125,
	ARTBW_50_25,
	ARTBW_100_50,
	ARTBW_200_100,
	ARTBW_400_200,
	ARTBW_800_400,
	ARTBW_1600_800,
	ARTBW_3200_1600  // 3200 Hz ODR, 1600 Hz bandwidth
};

enum eInterruptPin
{
	ADXL345_INT_1 = 0,
	ADXL345_INT_2 = 1
};
//--------------------------------------------------------------------
//-- ADXL345 STRUCTURES
//--------------------------------------------------------------------
struct sEvents
{
	bool isFreeFall;
	bool isSingleTap;
	bool isDoubleTap;
};

enum eEventName
{
	FREE_FALL = 0,
	SINGLE_TAP,
	DOUBLE_TAP
};

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

struct sOrientation
{
	float pitch;
	float roll;
};
#endif /* AXESSTRUCTURE */

//--------------------------------------------------------------------
//-- ADXL345 CLASS
//--------------------------------------------------------------------
class ADXL345 : public I2C_Sensor, public I2C_Interface
{
	public:
	ADXL345(String sensorName, uint8_t I2CAddress, uint8_t deviceVCCPin);
	
	eBeginStatus begin(void) override;
	void init(void) override;
	
	uint8_t getDeviceAddress(void) override;
	String getDeviceName() override;
	uint8_t getDeviceID(void) override;
	
	void read(void) override;
	
	void setDataRate(eADXL345_DatarateBandwidth datarateAndBandwidth);
	void setRange(eADXL345_Range range);

	float getResolution();
	sIntAxes getRaw(void);
	sFloatAxes getRawFiltered(void);
	sFloatAxes getScaled(void);
	sFloatAxes getNormalized(void);
	sOrientation getPitchRoll(void);

	bool getEventByName(eEventName eventName);

	private:
	const String _sensorName;
	const uint8_t _I2CAddress;
	const uint8_t _deviceVCCPin;
	
	float _resolution;
	
	sFloatAxes offset;
	sFloatAxes gain;
	eADXL345_Range _range;
	
	sIntAxes accelerationsRaw;
	sFloatAxes accelerationsRawFiltered;
	sFloatAxes accelerationsScaled;
	sFloatAxes accelerationsNormalized;
	sOrientation orientation;
	
	sEvents events;
	
	
	void powerOn(void);
	void resetPower(void);
	
	void lowPassFilter(float alpha);
	
	void calculatePitchRoll();
	
	// Power Control
	void setPowerControlRegister(uint8_t value);
	
	// Data Rate
	eADXL345_DatarateBandwidth readDataRateFromDevice(void);
	
	// Range
	eADXL345_Range readRangeFromDevice(void);
	void setDataFormatRegister(void);
	
	// FIFO
	void setControlFIFORegister(uint8_t value);
	
	// Interrupts
	void setFreeFall(void);
	void setFreeFallThresholdRegister(float accelerationInG);
	void setFreeFallTimeRegister(uint16_t timeinMS);
	void setSingleTap(void);
	void setDoubleTap(void);
	void setAxisControlForSingleDoubleTapRegister(uint8_t mask);
	void setTapThresholdRegister(float accelerationInG);
	void setTapDurationRegister(uint16_t timeinMS);
	void setDoubleTapWindowRegister(uint16_t timeInMS);
	void setDoubleTapLatencyRegister(uint16_t timeInMS);
	void setInterruptMappingControlRegister(eInterruptPin pin);
	void setInterruptEnableControlRegister(uint8_t mask);
	void getSourceOfInterrupts(void);
	
	// Calibration
	void useCompensationParameters(void);
	#if(ADXL345_ENABLE_CALIBRATION_PROCESS == 0x01)
	void performCalibrationProcess(void);
	#endif
};

//--------------------------------------------------------------------
//-- ADXL345 EXTERN OBJECT
//--------------------------------------------------------------------
extern ADXL345 Accelerometer;

#endif