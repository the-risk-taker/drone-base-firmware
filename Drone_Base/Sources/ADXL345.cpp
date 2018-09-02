/*
* ADXL345.cpp
*
* Created: 03.06.2018 21:31:22
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/


//--------------------------------------------------------------------
//-- ADXL345 INCLUDES
//--------------------------------------------------------------------
#include "../Headers/ADXL345.h"
#include "../Configs/Config.h"
#include "../Headers/Status.h"

//--------------------------------------------------------------------
//-- ADXL345 CLASS
//--------------------------------------------------------------------
ADXL345::ADXL345(String sensorName, uint8_t I2CAddress, uint8_t deviceVCCPin) : _sensorName(sensorName), _I2CAddress(I2CAddress), _deviceVCCPin(deviceVCCPin)
{
	// ADXL345 is 13bit, and can meassure up to +/- 16g -> (16*2)/(2^13), 
	this->_resolution = 0.00390625F;
	
#if( ADXL345_ENABLE_CALIBRATION_PROCESS == 0x01)
	this->offset.X = 0.0F;
	this->offset.Y = 0.0F;
	this->offset.Z = 0.0F;
	
	this->gain.X = 1.0F;
	this->gain.Y = 1.0F;
	this->gain.Z = 1.0F;
#else
	// Put here compensation parameters:
	this->offset.X = 4.12547;
	this->offset.Y = -4.03745;
	this->offset.Z = 25.33646;

	this->gain.X = 0.99608;
	this->gain.Y = 0.99793;
	this->gain.Z = 0.98863;
#endif
	
	events.isFreeFall = false;
	events.isSingleTap = false;
	events.isDoubleTap = false;
}

eBeginStatus ADXL345::begin(void)
{
	eBeginStatus returnedStatus = BEGIN_ERROR;
	
	this->powerOn();
	
	for(uint8_t interator = 0; interator < NUMBER_OF_INIT_RETRIES; interator++)
	{
		if(0xE5 == this->getDeviceID())
		{
			returnedStatus = BEGIN_COMPLETED;
			
			this->init();
			break;
		}
		
		Status.printMessage("I repeat the initialization of ADXL345 for the: " + String(interator) + " time");

		this->resetPower();
	}

	return returnedStatus;
}

void ADXL345::init(void)
{
	this->setPowerControlRegister(STANDBY_MODE);
	this->setPowerControlRegister(MEASURE_MODE);
	
	this->setRange(AFS_2G); // must be used with ADXL345::setDataFormatRegister();
	
	this->setDataRate(ARTBW_200_100);
	this->setControlFIFORegister(BAYPASS_FIFO);
	
	// Set Interrupts Parameters:
	this->setFreeFall();
	this->setSingleTap();
	this->setDoubleTap();
	
	// Map Interrupt Pins and Enable Desired Interrupts
	this->setInterruptMappingControlRegister(ADXL345_INT_1);
	this->setInterruptEnableControlRegister(ENABLE_FF_INT_MASK | ENABLE_ST_INT_MASK | ENABLE_DT_INT_MASK);
	
#if( ADXL345_ENABLE_CALIBRATION_PROCESS == 0x01)
	this->performCalibrationProcess();
#endif
}

String ADXL345::getDeviceName()
{
	return this->_sensorName;
}

uint8_t ADXL345::getDeviceID(void)
{
	/*
	Register Bitfields
		[7:0] DEVID
		
	The DEVID register holds a fixed device ID code of 0xE5 (345 octal).
	*/

	uint8_t deviceID;
	
	// Should return 0xE5;
	this->readByte(this->_I2CAddress, ADXL345_REG_ID, deviceID);
	
	return deviceID;
}

void ADXL345::setPowerControlRegister(uint8_t measureBit)
{
	/*
	POWER_CTL Register Bitfields
		[5]		LINK
		[4]		AUTO_SLEEP
		[3]		MEASURE :
			1 - MEASSURE MODE
			0 - STANDBY MODE
		[2]		SLEEP
		[1:0]	WAKEUP
	*/
	
	// Set bits [5], [4], [2:0] to zero.
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	registerValue |= measureBit;
	
	this->writeByte(this->_I2CAddress, ADXL345_POWER_CTL, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_POWER_CTL, this->readByte(this->_I2CAddress, ADXL345_POWER_CTL));
	#endif // ADXL345_DEBUG
}

void ADXL345::setDataRate(eADXL345_DatarateBandwidth datarateAndBandwidth)
{
	/*
	Register Bitfields
		[4]		LOW_POWER:
			0 - NORMAL POWER MODE
			1 - REDUCED POWER MODE
		[3:0]	RATE
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= NORMAL_POWER;
	registerValue |= datarateAndBandwidth;
	
	this->writeByte(this->_I2CAddress, ADXL345_BW_RATE, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_BW_RATE, this->readByte(this->_I2CAddress, ADXL345_BW_RATE));
	#endif // ADXL345_DEBUG
}

eADXL345_DatarateBandwidth ADXL345::readDataRateFromDevice(void)
{
	return (eADXL345_DatarateBandwidth)(this->readByte(this->_I2CAddress, ADXL345_BW_RATE) & DATA_RATE_MASK);
}

void ADXL345::setRange(eADXL345_Range range)
{
	_range = range;
	// You subsequently must execute ADXL345::setDataFormatRegister() to obtain effect.
	
	this->setDataFormatRegister();
}

eADXL345_Range ADXL345::readRangeFromDevice(void)
{
	return (eADXL345_Range)(readByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT) & RANGE_MASK);
}

void ADXL345::setDataFormatRegister(void)
{
	/*
	Register Bitfields
		[7]		SELF_TEST
		[6]		SPI
		[5]		INT_INVERT
		[3]		FULL_RES:
			0 - The device is in 10-bit mode, and the range bits determine the maximum g range and scale factor.
			1 - Sets full resolution mode -> regardless of the selected range resolution is always 0.004 [mg/LSB]
		[2]		JUSTIFY:
			0 - Left justified
			1 - Right justified
		[1:0]	RANGE
	*/
	
	// Set bits [7], [6], [5] to zero.
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= FULL_RESOLUTION_MODE;
	registerValue |= JUSTIFY_RIGHT;
	registerValue |= this->_range;
	
	this->writeByte(this->_I2CAddress, ADXL345_DATA_FORMAT, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_DATA_FORMAT, this->readByte(this->_I2CAddress, ADXL345_DATA_FORMAT));
	#endif // ADXL345_DEBUG
}

void ADXL345::setControlFIFORegister(uint8_t FIFOMode)
{
	/*
	Register Bitfields
		[7:6] FIFO_MODE:
		00 - FIFO is bypassed.
		01 - FIFO mode
		10 - Stream mode
		11 - Trigger mode
		[5] TRIGGER
		[4:0] SAMPLES
	*/
	
	// Set bits [5:0] to zero.
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= FIFOMode;
	
	this->writeByte(this->_I2CAddress, ADXL345_FIFO_CTL, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_FIFO_CTL, this->readByte(this->_I2CAddress, ADXL345_FIFO_CTL));
	#endif // ADXL345_DEBUG
}

void ADXL345::setFreeFall(void)
{
	this->setFreeFallThresholdRegister(0.50f);
	this->setFreeFallTimeRegister(100UL);
}

void ADXL345::setSingleTap(void)
{
	this->setAxisControlForSingleDoubleTapRegister(ENABLE_TAP_ON_X | ENABLE_TAP_ON_Y | ENABLE_TAP_ON_Z);
	this->setTapDurationRegister(10UL);
	this->setTapThresholdRegister(4.0f);
}

void ADXL345::setDoubleTap(void)
{
	this->setDoubleTapWindowRegister(300UL);
	this->setDoubleTapLatencyRegister(10UL);
}

void ADXL345::setFreeFallThresholdRegister(float accelerationInG)
{
	/*
	Register Bitfields
		[7:0] THRESH_FF
		
		The scale factor is 62.5 mg/LSB. Note that a value of 0 mg may result
		in undesirable behavior if the free-fall interrupt is enabled.
		Values between 300 mg and 600 mg (0?05 to 0?09) are recommende		
		accelerationInG can be between:
			a) >0 and <= 16
		accelerationInG is recommended to be between:
			a) >= 0.3 and <= 0.6
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= constrain((uint16_t)(accelerationInG/FF_THRESHOLD_SCALE_FACTOR), FF_THRESHOLD_MIN_VALUE, FF_THRESHOLD_MAX_VALUE);
	
	this->writeByte(this->_I2CAddress, ADXL345_THRESH_FF, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8Value(ADXL345_THRESH_FF, this->readByte(this->_I2CAddress, ADXL345_THRESH_FF)*FF_THRESHOLD_SCALE_FACTOR);
	#endif // ADXL345_DEBUG
}

void ADXL345::setFreeFallTimeRegister(uint16_t timeinMS)
{
	/*
	Register Bitfields
		[7:0] TIME_FF
		
		The TIME_FF register is eight bits and stores an unsigned time value representing
		the minimum time that the value of all axes must be less than THRESH_FF to generate
		a free-fall interrupt. The scale factor is 5 ms/LSB. A value of 0 may result
		in undesirable behavior if the free-fall interrupt is enabled. Values between
		100 ms and 350 ms (0?14 to 0?46) are recommende		
		timeinMS should can be between:
			a) >0 and <= 16
		timeinMS is recommended to be between:
			a) >= 100 and <= 350
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= constrain((uint16_t)(timeinMS/FF_TIME_SCALE_FACTOR), FF_TIME_MIN_VALUE, FF_TIME_MAX_VALUE);
	
	this->writeByte(this->_I2CAddress, ADXL345_TIME_FF, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8Value(ADXL345_TIME_FF, this->readByte(this->_I2CAddress, ADXL345_TIME_FF)*FF_TIME_SCALE_FACTOR);
	#endif // ADXL345_DEBUG
}

void ADXL345::setInterruptMappingControlRegister(eInterruptPin pin)
{
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;

	if(ADXL345_INT_1 == pin)
	{
		registerValue |= ALL_EVENTS_ON_INT_1_PIN;
	}
	else
	{
		registerValue |= ALL_EVENTS_ON_INT_2_PIN;
	}
	
	this->writeByte(this->_I2CAddress, ADXL345_INT_MAP, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_INT_MAP, this->readByte(this->_I2CAddress, ADXL345_INT_MAP));
	#endif // ADXL345_DEBUG
}

void ADXL345::setInterruptEnableControlRegister(uint8_t mask)
{
	/*
	Register Bitfields
		[7] DATA_READY
		[6] SINGLE_TAP
		[5] DOUBLE_TAP
		[4] ACTIVITY
		[3] INACTIVITY
		[2] FREE_FALL
		[1] WATERMARK
		[0] OVERRUN
		
	Controls which interrupts are enabled.
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= mask;
	
	this->writeByte(this->_I2CAddress, ADXL345_INT_ENABLE, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_INT_ENABLE, this->readByte(this->_I2CAddress, ADXL345_INT_ENABLE));
	#endif // ADXL345_DEBUG
}

void ADXL345::getSourceOfInterrupts(void)
{
	/*
	Register Bitfields
		[7] DATA_READY
		[6] SINGLE_TAP
		[5] DOUBLE_TAP
		[4] ACTIVITY
		[3] INACTIVITY
		[2] FREE_FALL
		[1] WATERMARK
		[0] OVERRUN
	*/
	
	uint8_t registerValue = this->readByte(this->_I2CAddress, ADXL345_INT_SOURCE);
	
	if(registerValue & FF_INTERRUPT_MASK) {events.isFreeFall = true;}
	if(registerValue & ST_INTERRUPT_MASK) {events.isSingleTap = true;}
	if(registerValue & DT_INTERRUPT_MASK) {events.isDoubleTap = true;}
}

void ADXL345::setAxisControlForSingleDoubleTapRegister(uint8_t mask)
{
	/*
	Register Bitfields
		[3] SUPPRESS
		[2] TAP_X
		[1] TAP_Y
		[0] TAP_Z
	
	Controls the tap detection participation settings for each axis, and double-tap suppression.
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= mask;
	
	this->writeByte(this->_I2CAddress, ADXL345_TAP_AXES, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8(ADXL345_TAP_AXES, this->readByte(this->_I2CAddress, ADXL345_TAP_AXES));
	#endif // ADXL345_DEBUG
}

void ADXL345::setTapThresholdRegister(float accelerationInG)
{
	/*
	Tap threshold (THRESH_TAP @ 0x1D)
	Register Bitfields
		[7:0] THRESH_TAP
	
	The THRESH_TAP register is eight bits and holds the threshold value for
	tap interrupts. The data format is unsigned, therefore, the magnitude of
	the tap event is compared with the value in THRESH_TAP for normal tap detection.
	The scale factor is 62.5 mg/LSB (that is, 0xFF = 16 g). A value of 0 may result
	in undesirable behavior if single tap/double tap interrupts are enabled.
	
	accelerationInG should be between:
		a) >0 and <= 16
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= constrain((uint16_t)(accelerationInG/TAP_THRESHOLD_SCALE_FACTOR), TAP_THRESHOLD_MIN_VALUE, TAP_THRESHOLD_MAX_VALUE);
	
	this->writeByte(this->_I2CAddress, ADXL345_THRESH_TAP, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8Value(ADXL345_THRESH_TAP, this->readByte(this->_I2CAddress, ADXL345_THRESH_TAP)*TAP_THRESHOLD_SCALE_FACTOR);
	#endif // ADXL345_DEBUG
}

void ADXL345::setTapDurationRegister(uint16_t timeinMS)
{
	/*
	Tap duration (DUR @ 0x21)
	Register Bitfields
		[7:0] DUR
	
	The DUR register is eight bits and contains an unsigned time value representing
	the maximum time that an event must be above the THRESH_TAP threshold to qualify
	as a tap event. The scale factor is 625 us/LSB. A value of 0 disables the single
	tap/ double tap functions.
	
	timeInMS =
		a) 0		- disables the single tap functions
		b) 159,375	- max value for duration parameter
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= constrain((uint16_t)(timeinMS/TAP_DUR_SCALE_FACTOR), TAP_DUR_MIN_VALUE, TAP_DUR_MAX_VALUE);
	
	this->writeByte(this->_I2CAddress, ADXL345_DUR, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8Value(ADXL345_DUR, this->readByte(this->_I2CAddress, ADXL345_DUR) * TAP_DUR_SCALE_FACTOR);
	#endif // ADXL345_DEBUG
}

void ADXL345::setDoubleTapWindowRegister(uint16_t timeInMS)
{
	/*
	Register Bitfields
		[7:0] WINDOW
	
	The window register is eight bits and contains an unsigned time value
	representing the amount of time after the expiration of the latency time
	(determined by the latent register) during which a second valid tap can begin.
	The scale factor is 1.25 ms/LSB. A value of 0 disables the double tap function.
	
	timeInMS =
		a) 0		- disables the single tap functions
		b) 318.75	- max value for window parameter
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= constrain((uint16_t)(timeInMS/DOUBLE_TAP_WINDOW_FACTOR), DOUBLE_TAP_WINDOW_MIN_VALUE, DOUBLE_TAP_WINDOW_MAX_VALUE);
	
	this->writeByte(this->_I2CAddress, ADXL345_WINDOW, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8Value(ADXL345_WINDOW, this->readByte(this->_I2CAddress, ADXL345_WINDOW) * DOUBLE_TAP_WINDOW_FACTOR);
	#endif // ADXL345_DEBUG
}

void ADXL345::setDoubleTapLatencyRegister(uint16_t timeInMS)
{
	/*
	Register Bitfields
		[7:0] LATENT
	
	The latent register is eight bits and contains an unsigned time value representing
	the wait time from the detection of a tap event to the start of the time window
	(defined by the window register) during which a possible second tap event can be
	detected. The scale factor is 1.25 ms/LSB. A value of 0 disables the double tap function.
	
	timeInMS =
		a) 0		- disables the single tap functions
		b) 318.75	- max value for latency parameter
	*/
	
	uint8_t registerValue = ADXL345_CLEAR_REGISTER;
	
	registerValue |= constrain((uint16_t)(timeInMS/DOUBLE_TAP_LATENT_FACTOR), DOUBLE_TAP_LATENT_MIN_VALUE, DOUBLE_TAP_LATENT_MAX_VALUE);
	
	this->writeByte(this->_I2CAddress, ADXL345_LATENT, registerValue);
	
	#if( ADXL345_DEBUG == 0x01)
	Status.printRegister8Value(ADXL345_LATENT, this->readByte(this->_I2CAddress, ADXL345_LATENT) * DOUBLE_TAP_LATENT_FACTOR);
	#endif // ADXL345_DEBUG
}

void ADXL345::read(void)
{
	this->getSourceOfInterrupts();
	
	uint8_t rawData[NUMBER_OF_REGISTERS];  // x/y/z accelerations register data stored here
	this->readBytes(this->_I2CAddress, ADXL345_DATAX0, NUMBER_OF_REGISTERS, &rawData[0]);
	
	this->accelerationsRaw.X = (((int16_t)rawData[X_HIGH_BYTE]) << 8) | rawData[X_LOW_BYTE];
	this->accelerationsRaw.Y = (((int16_t)rawData[Y_HIGH_BYTE]) << 8) | rawData[Y_LOW_BYTE];
	this->accelerationsRaw.Z = (((int16_t)rawData[Z_HIGH_BYTE]) << 8) | rawData[Z_LOW_BYTE];
	
#if( ADXL345_ENABLE_CALIBRATION_PROCESS == 0x01)
	this->useCompensationParameters();
#endif
}

void ADXL345::useCompensationParameters(void)
{
	this->accelerationsRaw.X = int16_t(((float)this->accelerationsRaw.X - this->offset.X) / this->gain.X);
	this->accelerationsRaw.Y = int16_t(((float)this->accelerationsRaw.Y - this->offset.Y) / this->gain.Y);
	this->accelerationsRaw.Z = int16_t(((float)this->accelerationsRaw.Z - this->offset.Z) / this->gain.Z);
}

uint8_t ADXL345::getDeviceAddress(void)
{
	return this->_I2CAddress;
}

sIntAxes ADXL345::getRaw(void)
{
	return this->accelerationsRaw;
}

sFloatAxes ADXL345::getRawFiltered(void)
{
	this->lowPassFilter(ADXL345_LOW_PASS_FILTER_ALPHA);

	return this->accelerationsRawFiltered;
}

sFloatAxes ADXL345::getScaled(void)
{
	this->accelerationsScaled.X = this->accelerationsRaw.X * this->_resolution;
	this->accelerationsScaled.Y = this->accelerationsRaw.Y * this->_resolution;
	this->accelerationsScaled.Z = this->accelerationsRaw.Z * this->_resolution;	
		
	return this->accelerationsScaled;
}
sFloatAxes ADXL345::getNormalized(void)
{
	this->accelerationsNormalized.X = this->accelerationsRaw.X * this->_resolution * GRAVITY_ON_EARTH;
	this->accelerationsNormalized.Y = this->accelerationsRaw.Y * this->_resolution * GRAVITY_ON_EARTH;
	this->accelerationsNormalized.Z = this->accelerationsRaw.Z * this->_resolution * GRAVITY_ON_EARTH;	

	return this->accelerationsNormalized;
}

void ADXL345::calculatePitchRoll()
{
	this->getScaled();
	
	this->orientation.pitch = (-1.0F)*(atan2(this->accelerationsScaled.X, (sqrt(this->accelerationsScaled.Y * this->accelerationsScaled.Y + this->accelerationsScaled.Z * this->accelerationsScaled.Z))) * 180.0) / PI;
	this->orientation.roll = (atan2(this->accelerationsScaled.Y, (sqrt(this->accelerationsScaled.X * this->accelerationsScaled.X + this->accelerationsScaled.Z * this->accelerationsScaled.Z))) * 180.0) / PI;
}

sOrientation ADXL345::getPitchRoll(void)
{
	this->calculatePitchRoll();
	
	return this->orientation;
}

float ADXL345::getResolution()
{
	return this->_resolution;
}

bool ADXL345::getEventByName(eEventName eventName)
{
	bool eventStatus = false;
	switch (eventName)
	{
		case FREE_FALL:
		{
			 eventStatus = this->events.isFreeFall;
			 this->events.isFreeFall = false;
		}
		break;
		case SINGLE_TAP:
		{
			eventStatus = this->events.isSingleTap;
			this->events.isSingleTap = false;
		}
		break;
		case DOUBLE_TAP:
		{
			eventStatus = this->events.isDoubleTap;
			this->events.isDoubleTap = false;
		}
		break;
		default:
		{
			Status.printMessage("You are trying to get unknown or unimplemented event!");
		}
		break;
	}
	
	return eventStatus;
}

#if(ADXL345_ENABLE_CALIBRATION_PROCESS == 0x01)
void ADXL345::performCalibrationProcess(void)
{
	/*
		More information:
		http://www.analog.com/media/en/technical-documentation/application-notes/AN-1057.pdf	pages 6-8
		http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf			page 30
	*/
	
	bool			runFlag					= true,
					cycleFlag				= false;
			
	uint32_t		numberOfSamples			= 0,
					previousTime			= 0,
					actualTime				= 0;
					
	int32_t			testedAxisSum			= 0;
	float			results[6]				= {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
			
	uint8_t			testedAxisNumber		= 0,
					minusPlusTrialCounter	= 0;
		
	const uint16_t	timeOfRecording			= 4000,
					timeOfWaiting			= 1000;
	
	while(runFlag)
	{
		if(cycleFlag == false)
		{
			cycleFlag = true;
			testedAxisSum = 0;
			numberOfSamples = 0;
			
			minusPlusTrialCounter++;
			
			if(minusPlusTrialCounter == 1){Status.printMessage("Put acc pointing X axis up!"); testedAxisNumber = 0;}
			if(minusPlusTrialCounter == 2){Status.printMessage("Put acc pointing X axis down!");}
			
			if(minusPlusTrialCounter == 3){Status.printMessage("Put acc pointing Y axis up!"); testedAxisNumber = 1;}
			if(minusPlusTrialCounter == 4){Status.printMessage("Put acc pointing Y axis down!");}
			
			if(minusPlusTrialCounter == 5){Status.printMessage("Put acc pointing Z axis up!"); testedAxisNumber = 2;}
			if(minusPlusTrialCounter == 6){Status.printMessage("Put acc pointing Z axis down!");}

			Status.printMessage("And then press key for example '3' when ready and press Enter.");
			Status.printMessage("... Or Press '1' key and Enter to finish! and get compensation parameters");

			// Wait for the key and enter.
			while (!Status.getUARTObject().available())
			{	
				if(Status.getUARTObject().parseInt() == 1)
				{
					runFlag = false;
				}

				this->read(); // Clear ADXL345 buffers.
			}
			
			Status.getUARTObject().println();
			
			previousTime = millis();
		}

		actualTime = millis();
		
		// Wait 1s (default) to settle.
		if((actualTime - previousTime) >= timeOfWaiting)
		{
			this->read();
			delay(5);

			int16_t currentSampleValue = 0;
			
			switch(testedAxisNumber)
			{
				case 0: // X
				{
					currentSampleValue = this->getRaw().X;
				}
				break;
				case 1: // Y
				{
					currentSampleValue = this->getRaw().Y;
				}
				break;
				case 2: // Z
				{
					currentSampleValue = this->getRaw().Z;
				}
				break;
			}
			
			testedAxisSum += currentSampleValue;
			numberOfSamples++;
			
			Status.printMessage(String(currentSampleValue) + "\t" + String(numberOfSamples) + "\t" + String((float)testedAxisSum/(float)numberOfSamples));
		}

		// After 4s (default) stop recording.
		if((actualTime - previousTime) >= (timeOfRecording + timeOfWaiting))
		{
			cycleFlag = false;
			
			results[minusPlusTrialCounter - 1] = (float)testedAxisSum/(float)numberOfSamples;
		}
		
		while(Status.getUARTObject().available()){Status.getUARTObject().read(); /* Clear buffer. */}
	}
	
	const uint16_t LSB = 256;
	
	this->offset.X = 0.5F * (results[0] + results[1]);
	this->gain.X = 0.5F * ((results[0] - results[1]) / (float)LSB);
	
	this->offset.Y = 0.5F * (results[2] + results[3]);
	this->gain.Y = 0.5F * ((results[2] - results[3]) / (float)LSB);
	
	this->offset.Z = 0.5F * (results[4] + results[5]);
	this->gain.Z = 0.5F * ((results[4] - results[5]) / (float)LSB);

	Status.printMessage("Copy this calibration parmeters to: 'ADXL345::ADXL345(String sensorName, ...)' constructor in #else statement.\n");
	
	Status.getUARTObject().println("this->offset.X = " + String(this->offset.X, 5) + ";");
	Status.getUARTObject().println("this->offset.Y = " + String(this->offset.Y, 5) + ";");
	Status.getUARTObject().println("this->offset.Z = " + String(this->offset.Z, 5) + ";\n");
	
	Status.getUARTObject().println("this->gain.X = " + String(this->gain.X, 5) + ";");
	Status.getUARTObject().println("this->gain.Y = " + String(this->gain.Y, 5) + ";");
	Status.getUARTObject().println("this->gain.Z = " + String(this->gain.Z, 5) + ";\n");
		
	Status.printMessage("Then change ADXL345_ENABLE_CALIBRATION_PROCESS define to 0x00");
	
	while(1){}
}
#endif

void ADXL345::lowPassFilter(float alpha)
{
	this->accelerationsRawFiltered.X = alpha * (float)this->accelerationsRaw.X + (1.0 - alpha) * (float)this->accelerationsRawFiltered.X;
	this->accelerationsRawFiltered.Y = alpha * (float)this->accelerationsRaw.Y + (1.0 - alpha) * (float)this->accelerationsRawFiltered.Y;
	this->accelerationsRawFiltered.Z = alpha * (float)this->accelerationsRaw.Z + (1.0 - alpha) * (float)this->accelerationsRawFiltered.Z;
}

void ADXL345::powerOn(void)
{
	pinMode(this->_deviceVCCPin, OUTPUT);
	digitalWrite(this->_deviceVCCPin, HIGH);
	delay(100);
}

void ADXL345::resetPower(void)
{
	digitalWrite(this->_deviceVCCPin, LOW);
	delay(100);
	digitalWrite(this->_deviceVCCPin, HIGH);
	delay(100);
}

//--------------------------------------------------------------------
//-- ADXL345 EXTERN OBJECT
//--------------------------------------------------------------------
ADXL345 Accelerometer("ADXL345", ADXL345_ADDRESS, ADXL345_VCC_PIN);