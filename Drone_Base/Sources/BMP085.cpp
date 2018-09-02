/* 
* BMP085.cpp
*
* Created: 06.06.2018 21:21:22
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

//--------------------------------------------------------------------
//-- BMP085 INCLUDES
//--------------------------------------------------------------------
#include "../Headers/BMP085.h"
#include "../Headers/Status.h"

//--------------------------------------------------------------------
//-- BMP085 CLASS
//--------------------------------------------------------------------
 BMP085::BMP085(String sensorName, uint8_t I2CAddress) : _sensorName(sensorName), _I2CAddress(I2CAddress) 
{
}

eBeginStatus BMP085::begin(void)
{
	eBeginStatus returnedStatus = BEGIN_ERROR;

	if(0x55 == this->getDeviceID())
	{
		if(this->readCalibrationCoefficients())
		{
			this->init();
			returnedStatus = BEGIN_COMPLETED;
		}
	}	

	return returnedStatus;
}

void BMP085::init(void)
{
	this->setMode(ULTRA_HIGH_RESOLUTION);
}

uint8_t BMP085::getDeviceAddress(void)
{
	return this->_I2CAddress;
}

String BMP085::getDeviceName(void)
{
	return this->_sensorName;
}

void BMP085::setMode(eBMP085_Mode mode)
{
	this->_mode = mode;
}

uint8_t BMP085::getDeviceID(void)
{
	uint8_t deviceID;
	
	// Should return 0x55;
	this->readByte(this->_I2CAddress, BMP085_REG_ID, deviceID);
	
	return deviceID;
}

bool BMP085::readCalibrationCoefficients(void)
{
	/*
		The 176 bit E2PROM is partitioned in 11 words of 16 bit each. These contain 11 calibration
		coefficients. Every sensor module has individual coefficients. Before the first calculation of
		temperature and pressure, the master reads out the E2PROM data.

		The data communication can be checked by checking that none of the words has the value 0 or
		0xFFFF.
			Parameter	MSB		LSB		Type
			AC1			0xAA	0xAB	int16_t
			AC2			0xAC	0xAD	int16_t
			AC3			0xAE	0xAF	int16_t
			AC4			0xB0	0xB1	uint16_t
			AC5			0xB2	0xB3	uint16_t
			AC6			0xB4	0xB5	uint16_t
			B1			0xB6	0xB7	int16_t
			B2			0xB8	0xB9	int16_t
			MB			0xBA	0xBB	int16_t
			MC			0xBC	0xBD	int16_t
			MD			0xBE	0xBF	int16_t
			
		My module data:
			64:	7753
			64:	-1170
			64:	-14742
			65:	33308
			66:	24282
			67:	15627
			67:	5498
			68:	62
			69:	-32768
			70:	-11075
			71:	2432
	*/
	
	bool returnedStatus = false;
	
	this->BMP085_AC1 = (int16_t)this->read16(BMP085_AC1_MSB, BMP085_AC1_LSB);
	this->BMP085_AC2 = (int16_t)this->read16(BMP085_AC2_MSB, BMP085_AC2_LSB);
	this->BMP085_AC3 = (int16_t)this->read16(BMP085_AC3_MSB, BMP085_AC3_LSB);
	this->BMP085_AC4 = this->read16(BMP085_AC4_MSB, BMP085_AC4_LSB);
	this->BMP085_AC5 = this->read16(BMP085_AC5_MSB, BMP085_AC5_LSB);
	this->BMP085_AC6 = this->read16(BMP085_AC6_MSB, BMP085_AC6_LSB);
	this->BMP085_B1 = (int16_t)this->read16(BMP085_B1_MSB, BMP085_B1_LSB);
	this->BMP085_B2 = (int16_t)this->read16(BMP085_B2_MSB, BMP085_B2_LSB);
	this->BMP085_MB = (int16_t)this->read16(BMP085_MB_MSB, BMP085_MB_LSB);
	this->BMP085_MC = (int16_t)this->read16(BMP085_MC_MSB, BMP085_MC_LSB);
	this->BMP085_MD = (int16_t)this->read16(BMP085_MD_MSB, BMP085_MD_LSB);
	
	// Check connection data should be not equal to 0 or 0xFF (0xFFFF for 16 bit)
	if(this->BMP085_MD != 0x0000 || this->BMP085_MD != 0xFFFF)
	{
		returnedStatus = true;
	}
	
	return returnedStatus;
}

bool BMP085::readUncompensatedTemperature(void)
{
	/*
		According to BMP085 DataSheet p.13:
		Temperature (0x2E) max. conversion time [ms] = 4.5

		Write 0x2E into reg 0xF4, wait 4.5ms
		Read reg 0xF6 (MSB), 0xF7 (LSB)
	*/
	
	bool readoutComplete = false;
	static bool startReadoutFlag = false;
	static uint32_t	previousTime;
	
	if(startReadoutFlag == false)
	{
		// send request
		this->writeByte(this->_I2CAddress, BMP085_REG_CONTROL, BMP085_REG_GET_TEMP);
		
		previousTime = millis();
		startReadoutFlag = true;
	}		
	
	if((millis() - previousTime) >= 5)
	{
		// if conversion finished, read data
		this->BMP085_UT = (int32_t)this->read16(BMP085_TEMPERATURE_MSB, BMP085_TEMPERATURE_LSB);
		startReadoutFlag = false;
		readoutComplete = true;
	}
	
	return readoutComplete;
}

void BMP085::calculateTemperature(void)
{
	/*
		According to BMP085 DataSheet p.13:
	*/
	
	int32_t		X1,
				X2;
				
	X1 = ((this->BMP085_UT - (int32_t)this->BMP085_AC6) * (int32_t)this->BMP085_AC5) >> 15;
	X2 = ((int32_t)this->BMP085_MC << 11) / (X1 + (int32_t)this->BMP085_MD);
	this->BMP085_B5 = X1 + X2;
	this->temperature = (float)((this->BMP085_B5 + 8) >> 4);		// Temperature in 0.1C
	this->temperature /= 10.0F;										// Temperature in 1C
}

float BMP085::getTemperature(void)
{
	return this->temperature;
}

uint8_t BMP085::getTimeInterval(void)
{
	/*
		According to BMP085 DataSheet p.13:
		Pressure  max. conversion time [ms]: 
			(osrs = 0) 0x34 4.5
			(osrs = 1) 0x74 7.5
			(osrs = 2) 0xB4 13.5
			(osrs = 3) 0xF4 25.5
	*/
	
	uint8_t intervalTime = 0;
	
	switch(this->_mode)
	{
		case ULTRA_LOW_POWER:
		{
			intervalTime = 5;
		}
		break;
		case STANDARD:
		{
			intervalTime = 8;
		}
		break;
		case HIGH_RESOLUTION:
		{
			intervalTime = 14;
		}
		break;
		case ULTRA_HIGH_RESOLUTION:
		{
			intervalTime = 26;
		}
		break;
	}
	
	return intervalTime;
}

bool BMP085::readUncompensatedPressure(void)
{
	/*
		According to BMP085 DataSheet p.13:
		Pressure (0x34 + (oss<<6)) max. conversion time [ms] = 
			(osrs = 0) 0x34 4.5
			(osrs = 1) 0x74 7.5
			(osrs = 2) 0xB4 13.5
			(osrs = 3) 0xF4 25.5
			
		Write 0x34+(oss<<6) into reg 0xF4, wait according to osrs x ms.
		Read reg 0xF6 (MSB), 0xF7 (LSB), 0xF8 (XLSB)
	*/
	
	bool readoutComplete = false;
	static bool startReadoutFlag = false;
	static uint32_t	previousTime;
	
	if(startReadoutFlag == false)
	{
		// send request
		this->writeByte(this->_I2CAddress, BMP085_REG_CONTROL, (BMP085_REG_GET_PRESSURE | (this->_mode << 6)));
		
		previousTime = millis();
		startReadoutFlag = true;
	}
	
	if((millis() - previousTime) >= this->getTimeInterval())
	{
		// if conversion finished, read data
		this->BMP085_UP = (this->readByte(this->_I2CAddress, BMP085_PRESSURE_MSB) << 16);
		this->BMP085_UP |= (this->readByte(this->_I2CAddress, BMP085_PRESSURE_LSB) << 8);
		this->BMP085_UP |= this->readByte(this->_I2CAddress, BMP085_PRESSURE_XLSB);
		this->BMP085_UP >>= (8 - this->_mode);
		
		startReadoutFlag = false;
		readoutComplete = true;
	}
	
	return readoutComplete;	
}

void BMP085::calculatePressure(void)
{
	/*
		According to BMP085 DataSheet p.13:
	*/
	
	int32_t		B6,
				X1,
				X2,
				X3,
				B3,
				tempPressure;
				
	uint32_t	B4,
				B7;
	
	B6 = this->BMP085_B5 - 4000;
	X1 = ((int32_t)this->BMP085_B2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (int32_t)this->BMP085_AC2 * B6 >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)this->BMP085_AC1 * 4 + X3) << this->_mode) + 2) >> 2;
 
	X1 = (int32_t)this->BMP085_AC3 * B6 >> 13;
	X2 = ((int32_t)this->BMP085_B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (uint32_t)this->BMP085_AC4 * (uint32_t)(X3 + 32768) >> 15;
	B7 = ((uint32_t)this->BMP085_UP - B3) * (uint32_t)(50000UL >> this->_mode);
	
	if(B7 < 0x80000000)
	{
		tempPressure = (B7 * 2) / B4;
	}
	else
	{
		tempPressure = (B7/B4) * 2;
	}
	
	X1 = (tempPressure >> 8) * (tempPressure >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * tempPressure) >> 16;
	
	tempPressure = tempPressure + ((X1 + X2 + (int32_t)3791) >> 4); // Pressure in Pa
	
	#if( ENABLE_MOVING_AVERAGE_FILTERING == 0x01)
		this->pressure = this->movingAverage(tempPressure);
	#else
		this->pressure = tempPressure;
	#endif // ENABLE_MOVING_AVERAGE_FILTERING
}

int32_t BMP085::getPressure(void)
{
	return this->pressure;
}

void BMP085::read(void)
{
	/*
		To get pressure it is needed to first request temperature data and wait
		next read temperature data, perform some calculation,
		then request pressure data and wait, then read pressure data.
		
		Temperature data is needed to calculate pressure.
	*/
	
	static bool temperatureReadoutEnableFlag = true,
				pressureReadoutEnableFlag = false;
	
	if(temperatureReadoutEnableFlag)
	{
		// Wait for conversion complete:
		if(this->readUncompensatedTemperature())
		{
			// Calculate temperature
			this->calculateTemperature();
			
			// Enable pressure readout.
			pressureReadoutEnableFlag = true;
			temperatureReadoutEnableFlag = false;
		}
	}
	
	if(pressureReadoutEnableFlag)
	{
		// Wait for conversion complete:
		if(this->readUncompensatedPressure())
		{
			// Calculate pressure.
			this->calculatePressure();
			pressureReadoutEnableFlag = false;
			temperatureReadoutEnableFlag = true;
		}
	}
}

void BMP085::calculateAltitude(uint32_t seaLevelPressure)
{
	/*
		According to BMP085 DataSheet p.14:
		
		(1/5,255) = 0,19029495718363463368220742150333;
	*/

	this->altitude = (float)(44330 * (1.0 - pow(this->pressure / (float)seaLevelPressure, 0.190295)));
}

float BMP085::getAltitude(void)
{
	this->calculateAltitude();
	return this->altitude;
}

uint16_t BMP085::read16(uint8_t regMSB, uint8_t regLSB)
{
	return ((this->readByte(this->_I2CAddress, regMSB) << 8) | this->readByte(this->_I2CAddress, regLSB));
}

#if( ENABLE_MOVING_AVERAGE_FILTERING == 0x01)
int32_t BMP085::movingAverage(int32_t newValue)
{
	const uint8_t bufferSize = BMP085_MOVING_AVERAGE_RANGE;
	static int32_t buffer[bufferSize];
	int64_t sum = 0;
	
	buffer[0] = newValue;
	
	for(uint8_t i = 0; i < bufferSize; i++)
	{
		sum += buffer[i];
	}
	
	this->moveElements(buffer, bufferSize - 1);

	return (int32_t)(sum/bufferSize);
}

void BMP085::moveElements(int32_t table[], uint8_t lastElementIndex)
{
	for(uint8_t i = lastElementIndex; i > 0; i--)
	{
		table[i] = table[i - 1];
	}
}
#endif

//--------------------------------------------------------------------
//-- BMP085 EXTERN OBJECT
//--------------------------------------------------------------------
 BMP085 Barometer("BMP085", BMP085_ADDRESS);