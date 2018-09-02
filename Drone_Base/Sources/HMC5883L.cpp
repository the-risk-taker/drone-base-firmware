/* 
* HMC5883L.cpp
*
* Created: 12.06.2018 11:19:50
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

//--------------------------------------------------------------------
//-- HMC5883L INCLUDES
//--------------------------------------------------------------------
#include "../Headers/HMC5883L.h"
#include "../Headers/Status.h"
#include "../Configs/Config.h"

//--------------------------------------------------------------------
//-- HMC5883L CLASS
//--------------------------------------------------------------------
 HMC5883L::HMC5883L(String sensorName, uint8_t I2CAddress, uint8_t dataReadyPin) : _sensorName(sensorName), _I2CAddress(I2CAddress), _dataReadyPin(dataReadyPin)
 {
	 this->singleModeStatus = DISABLED;
	 this->selftTestStatus = FAILED;
	 
	 this->gainFactor.X = 1.0F;
	 this->gainFactor.Y = 1.0F;
	 this->gainFactor.Z = 1.0F;
	 
	 this->offsetFactor.X = 0.0F;
	 this->offsetFactor.Y = 0.0F;
	 this->offsetFactor.Z = 0.0F;
 }


eBeginStatus HMC5883L::begin(void)
{
	eBeginStatus returnedStatus = BEGIN_ERROR;
	
	if(('H'| '4' | '3') == this->getDeviceID())
	{
		this->setExternalInterrupts();
		this->performSelfTestCalibration();
		this->init();

		returnedStatus = BEGIN_COMPLETED;
	}
	
	return returnedStatus;
}

void HMC5883L::init(void)
{
	this->setConfigurationRegisterA(NORMAL_MEASUREMENT, ODR_75_HZ, SAMPLES_8);
	this->setConfigurationRegisterB(RESOLUTION_0_73);
	this->setModeRegister(SINGLE_MEASUREMENT_MODE);
	
	// Discard unreliable data after parameters change.
	for(uint8_t i = 0; i < 2; i++)
	{
		this->read();
		while(!this->getDataReadyFlag()){}
	}
}

void HMC5883L::read(void)
{
	uint8_t rawData[NUMBER_OF_REGISTERS];  // x/y/z magnetometer register data stored here
	this->readBytes(this->_I2CAddress, HMC5883L_OUT_X_MSB, NUMBER_OF_REGISTERS, &rawData[0]);
	
	this->rawData.X = ((int16_t)rawData[HMC5883L_X_HIGH_BYTE] << 8) | rawData[HMC5883L_X_LOW_BYTE];
	this->rawData.Y = ((int16_t)rawData[HMC5883L_Y_HIGH_BYTE] << 8) | rawData[HMC5883L_Y_LOW_BYTE];
	this->rawData.Z = ((int16_t)rawData[HMC5883L_Z_HIGH_BYTE] << 8) | rawData[HMC5883L_Z_LOW_BYTE];

	// Use offsets and scales from Self Test measurements in raw data meassurements:
	this->useSelfTestCompensationParameters();

	if(ENABLED == this->singleModeStatus)
	{
		// Need to do this, because after single mode device is setting to idle mode. 
		this->setModeRegister(SINGLE_MEASUREMENT_MODE);
	}
}

String HMC5883L::getDeviceName(void)
{
	return this->_sensorName;
}

uint8_t HMC5883L::getDeviceAddress(void)
{
	return this->_I2CAddress;
}

uint8_t HMC5883L::getDeviceID(void)
{
	uint8_t returnedValue	= 0,
			tempData		= 0;
	
	this->readByte(this->_I2CAddress, HMC5883L_IDENTIFICATION_A, tempData); // Should return H (ASCI)
	returnedValue |= tempData;
	this->readByte(this->_I2CAddress, HMC5883L_IDENTIFICATION_B, tempData); // Should return 4 (ASCI)
	returnedValue |= tempData;
	this->readByte(this->_I2CAddress, HMC5883L_IDENTIFICATION_C, tempData); // Should return 3 (ASCI)
	returnedValue |= tempData;
	
	return returnedValue;
}

bool HMC5883L::getDataReadyFlag(uint8_t timeout)
{
	static uint32_t previousTime = millis();

	// Make copy:
	bool tempDataReadyFlag = this->dataReadyFlag;
	
	if(tempDataReadyFlag)
	{
		this->dataReadyFlag = false;
		
		previousTime = millis();
	}
	// Prevent from unreaded DRDY interrupt:
	else if((millis() - previousTime) >= timeout)
	{
		tempDataReadyFlag = true;
		
		previousTime = millis();
		Status.printWarningMesage("Data Ready Flag reseted!");
	}
	
	return tempDataReadyFlag;
}

sIntAxes HMC5883L::getRaw(void)
{
	return this->rawData;
}

sFloatAxes HMC5883L::getScaled(void)
{
	this->scaledData.X = (float)this->rawData.X * this->_resolution;
	this->scaledData.Y = (float)this->rawData.Y * this->_resolution;
	this->scaledData.Z = (float)this->rawData.Z * this->_resolution;
	
	// Use hard and soft calibration parameters in scaled measurements:
	this->useHardAndSoftIronCompensationParameters();
	
	return this->scaledData;
}

void HMC5883L::setResolution(eHMC5883LResolution resolution)
{
	switch(resolution)
	{
		case RESOLUTION_0_73:
		{
			this->_resolution = 0.73F; // 0.73mGa
		}
		break;
		case RESOLUTION_0_92:
		{
			this->_resolution = 0.92F; // 0.92mGa
		}
		break;
		case RESOLUTION_1_22:
		{
			this->_resolution = 1.22F; // 1.22mGa
		}
		break;
		case RESOLUTION_1_52:
		{
			this->_resolution = 1.52F; // 1.52mGa
		}
		break;
		case RESOLUTION_2_27:
		{
			this->_resolution = 2.27F; // 2.27mGa
		}
		break;
		case RESOLUTION_2_56:
		{
			this->_resolution = 2.56F; // 2.56mGa
		}
		break;
		case RESOLUTION_3_03:
		{
			this->_resolution = 3.03F; // 3.03mGa
		}
		break;
		case RESOLUTION_4_35:
		{
			this->_resolution = 4.35F; // 4.35mGa
		}
		break;
	}
}

uint16_t HMC5883L::getGainLSBperGauss(eHMC5883LResolution resolution)
{
	uint16_t gain = 0;
	
	switch (resolution)
		{
		case RESOLUTION_0_73:
		{
			gain = 1370;
		}
		break;
		case RESOLUTION_0_92:
		{
			gain = 1090;
		}
		break;
		case RESOLUTION_1_22:
		{
			gain = 820;
		}
		break;
		case RESOLUTION_1_52:
		{
			gain = 660;
		}
		break;
		case RESOLUTION_2_27:
		{
			gain = 440;
		}
		break;
		case RESOLUTION_2_56:
		{
			gain = 390;
		}
		break;
		case RESOLUTION_3_03:
		{
			gain = 330;
		}
		break;
		case RESOLUTION_4_35:
		{
			gain = 230;
		}
		break;
	}
	
	return gain;
}

void HMC5883L::setExternalInterrupts(void)
{
	/*
		IMPORTATNT:
		pinMode(pin, INPUT_PULLUP) must be performed before attachInterrupt() function.
	*/
	
	pinMode(this->_dataReadyPin, INPUT_PULLUP);
	 
	switch (this->_dataReadyPin)
	{
		case HMC5883L_A_DATA_READY_PIN:
		{
			attachInterrupt(this->_dataReadyPin, HMC5883L::isrA, FALLING);
			// Remember object pointer
			 HMC5883L::instanceA = this;
		}
		break;
	#if (HMC5883L_ENABLE_MORE_INSTANCES == 0x01)
		case HMC5883L_B_DATA_READY_PIN:
		{
			attachInterrupt(this->_dataReadyPin, HMC5883L::isrB, FALLING);
			// Remember object pointer
			 HMC5883L::instanceB = this;
		}
		break;
	#endif
	}
}

void HMC5883L::setConfigurationRegisterA(eHMC5883LMeasurementMode measurementMode, eHMC5883LOutputDataRate outputDataRate, eHMC5883LAverageSamples samplesToAverage)
{
	/*
	Register Bitfields
		[7] CRA7			- Reserved
		[6:5] AA1|MA0		- Select number of samples averaged (1 to 8) per 
		measurement output.
				00 - 1 (default)
				01 - 2
				10 - 4
				11 - 8
		[4:2] DO2|D01|DO0	- Data Output Rate Bits. These bits set the rate
		at which data is written to all three data output registers.
		All three channels shall be measured within a given output rate.
		Other output rates with maximum rate of 160 Hz can be achieved by
		monitoring DRDY interrupt pin in single measurement mode.
					  (Hz)
				000 - 0.75
				001 - 1.5
				010 - 3.0
				011 - 7.5
				100 - 15 (default)
				101 - 30
				110 - 75
				111 - Reserved
				
		[1:0] MS1|MS0		- Measurement Configuration Bits. These bits define 
		the measurement flow of the device, specifically whether or not to 
		incorporate an applied bias into the measurement
				00 - Normal measurement configuration (Default). In normal measurement 
					configuration the device follows normal measurement flow. The positive
					and negative pins of the resistive load are left floating and high impedance.
				01 - Positive bias configuration for X, Y, and Z axes. In this configuration,
					a positive current is forced across the resistive load for all three axes.
				10 - Negative bias configuration for X, Y and Z axes. In this configuration,
					a negative current is forced across the resistive load for all three axes..
				11 - This configuration is reserved.
	*/
	
	uint8_t registerValue = HMC5883L_CLEAR_REGISTER;
	
	registerValue |= measurementMode;
	registerValue |= outputDataRate << 2;
	registerValue |= samplesToAverage << 5;
	//Status.printRegister8(HMC5883L_CONFIGURATION_A, registerValue);
	this->writeByte(this->_I2CAddress, HMC5883L_CONFIGURATION_A, registerValue);
	//Status.printRegister8(HMC5883L_CONFIGURATION_A, this->readByte(this->_I2CAddress, HMC5883L_CONFIGURATION_A));
}

void HMC5883L::setConfigurationRegisterB(eHMC5883LResolution resolution)
{
	/*
	Register Bitfields
		[7:5] GN2|GN1|GN0 - Gain Configuration Bits. These bits configure the gain 
		for the device. The gain configuration is common for all channels.
					   Field Range [Ga]	  Digital Resolution [mGa/LSB]
				 000 - +/- 0.88			- 0.73
				 001 - +/- 1.3			- 0.92 (Default)
				 010 - +/- 1.9			- 1.22
				 011 - +/- 2.5			- 1.52
				 100 - +/- 4.0			- 2.27
				 101 - +/- 4.7			- 2.56
				 110 - +/- 5.6			- 3.03
				 111 - +/- 8.1			- 4.35
				 
		[4:0]				- Reserved
		
		Choose a lower gain value (higher GN#) when total field strength causes overflow
		in one of the data output registers (saturation). Note that the very first
		measurement after a gain change maintains the same gain as the previous setting.
		The new gain setting is effective from the second measurement and on.
	*/
	
	uint8_t registerValue = HMC5883L_CLEAR_REGISTER;
	
	registerValue |= resolution << 5;
	this->setResolution(resolution);
	
	//Status.printRegister8(HMC5883L_CONFIGURATION_B, registerValue);
	this->writeByte(this->_I2CAddress, HMC5883L_CONFIGURATION_B, registerValue);
	//Status.printRegister8(HMC5883L_CONFIGURATION_B, this->readByte(this->_I2CAddress, HMC5883L_CONFIGURATION_B));
}

void HMC5883L::setModeRegister(eHMC5883LOperationMode operationMode)
{
	/*
	Register Bitfields
		[7:2] MR7:MR2 - set bit 7 (HS) to enable high speed I2C mode 3400kHz
				 
		[1:0] MD1|MD0 - Mode Select Bits. These bits select the operation mode of this device.
				00 - Continuous-Measurement Mode. In continuous-measurement mode, the device 
					continuously performs measurements and places the result in the data register.
					RDY goes high when new data is placed in all three registers. After a power-on
					or a write to the mode or configuration register, the first measurement set is
					available from all three data output registers after a period of 2/fDO and
					subsequent measurements are available at a frequency of fDO, where fDO is the
					frequency of data output.
				01 - Single-Measurement Mode (Default). When single-measurement mode is selected,
					device performs a single measurement, sets RDY high and returned to idle mode.
					Mode register returns to idle mode bit values. The measurement remains in the
					data output register and RDY remains high until the data output register is read
					or another measurement is performed.
				10 - Idle Mode. Device is placed in idle mode.
				11 - As above
		
		Choose a lower gain value (higher GN#) when total field strength causes overflow
		in one of the data output registers (saturation). Note that the very first
		measurement after a gain change maintains the same gain as the previous setting.
		The new gain setting is effective from the second measurement and on.
	*/
	
	uint8_t registerValue = HMC5883L_CLEAR_REGISTER;
	
	registerValue |= ENABLE_HIGH_SPEED_MODE; // mask = 0x80, speed = 3400kHz
	registerValue |= operationMode;
	
	//Status.printRegister8(HMC5883L_MODE, registerValue);
	this->writeByte(this->_I2CAddress, HMC5883L_MODE, registerValue);
	//Status.printRegister8(HMC5883L_MODE, this->readByte(this->_I2CAddress, HMC5883L_MODE));

	if(operationMode == SINGLE_MEASUREMENT_MODE)
	{
		this->singleModeStatus = ENABLED;
	}
	else
	{
		this->singleModeStatus = DISABLED;
	}
}

void HMC5883L::performSelfTestCalibration(void)
{
	/* 
		According to datasheet page 19 - SELF TEST OPERATION
	*/
	
	eHMC5883LResolution selfTestResolution = RESOLUTION_2_56;
	
	float			lowLimit = (float)this->getGainLSBperGauss(selfTestResolution) * HMC5883L_LOW_LIMIT,
					highLimit = (float)this->getGainLSBperGauss(selfTestResolution) * HMC5883L_HIGH_LIMIT;

	int32_t			sumXpos = 0,
					sumYpos = 0,
					sumZpos = 0,
					sumXneg = 0,
					sumYneg = 0,
					sumZneg = 0;
	
	const uint8_t	numberOfSamples = 13,
					numberOfOmmitedSamples = 3;
	
	uint8_t			counterPositiveBias = 0,
					counterNegativeBias = 0,
					testPassedCounter = 0,
					numberOfTestSamples = numberOfSamples - numberOfOmmitedSamples;
	
#if(HMC5883L_CALIBRATION_DEBUG == 0x01)	
	Status.printMessage("Self Test Started");	
	Status.printMessage("Self Test Margins for gain = " + String(this->getGainLSBperGauss(selfTestResolution)) + 
						"\n\tLow limit = " + String(lowLimit) + "\tHigh limit = " + String(highLimit));
	Status.printMessage("Number of test samples = " + String(numberOfTestSamples));
#endif

	this->setConfigurationRegisterA(POSITIVE_BIAS, ODR_15_HZ, SAMPLES_8);
	this->setConfigurationRegisterB(selfTestResolution);
	this->setModeRegister(SINGLE_MEASUREMENT_MODE);
	
#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
	Status.printMessage("Starting Self Test POSITIVE bias, samples:");
#endif
	// Perform test numberOfSamples times for POSITIVE bias.
	for(int i = 0; i < numberOfSamples; i++)
	{
		this->read();
		// Wait for interrupt
		while(!this->getDataReadyFlag()){}
		
		if(i >= numberOfOmmitedSamples)
		{
			// Check limits
			if(	(this->rawData.X > lowLimit) && (this->rawData.X < highLimit) &&
				(this->rawData.Y > lowLimit) && (this->rawData.Y < highLimit) &&
				(this->rawData.Z > lowLimit) && (this->rawData.Z < highLimit))
			{
				counterPositiveBias++;
				
			#if(HMC5883L_CALIBRATION_DEBUG == 0x01)	
				Status.printMessage(String(this->rawData.X) + "\t" + String(this->rawData.Y) + "\t" + String(this->rawData.Z));
			#endif
			
				sumXpos += this->rawData.X;
				sumYpos += this->rawData.Y;
				sumZpos += this->rawData.Z;
			}
		}
	}
	
	// Some 2-3 first readout can be corrupted due to gain change see setConfigurationRegisterB().
	if(counterPositiveBias == numberOfSamples - numberOfOmmitedSamples){testPassedCounter++;}
	
	this->setConfigurationRegisterA(NEGATIVIE_BIAS, ODR_15_HZ, SAMPLES_8);
	this->setConfigurationRegisterB(selfTestResolution);
	this->setModeRegister(SINGLE_MEASUREMENT_MODE);
	
#if(HMC5883L_CALIBRATION_DEBUG == 0x01)	
	Status.printMessage("Starting Self Test NEGATIVE bias, samples:");
#endif

	// Perform test numberOfSamples times for NEGATIVE bias.
	for(int i = 0; i < numberOfSamples; i++)
	{
		this->read();
		// Wait for interrupt
		while(!this->getDataReadyFlag()){}
		
		if(i >= numberOfOmmitedSamples)
		{
			// Check limits
			if(	(this->rawData.X > -highLimit) && (this->rawData.X < -lowLimit) &&
				(this->rawData.Y > -highLimit) && (this->rawData.Y < -lowLimit) &&
				(this->rawData.Z > -highLimit) && (this->rawData.Z < -lowLimit))
			{
				counterNegativeBias++;
				
			#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
				Status.printMessage(String(this->rawData.X) + "\t" + String(this->rawData.Y) + "\t" + String(this->rawData.Z));
			#endif
				
				sumXneg += this->rawData.X;
				sumYneg += this->rawData.Y;
				sumZneg += this->rawData.Z;
			}
		}
	}
	
	// Some 2-3 first readout can be corrupted due to gain change see setConfigurationRegisterB().
	if(counterNegativeBias == numberOfSamples - numberOfOmmitedSamples){testPassedCounter++;}
	
	if(testPassedCounter == 2)
	{
		this->selftTestStatus = PASSED;
		
	#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
		Status.printMessage(this->getDeviceName() + " Self Test Passed!");
	#endif
	}
	
	// Printing basic parameters:
#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
	Status.printMessage("Mean X pos = " + String((float)(sumXpos) / (float)(numberOfTestSamples)) +
						"\tMean Y pos = " + String((sumYpos) / (float)(numberOfTestSamples)) +
						"\tMean Z pos = " + String((sumZpos) / (float)(numberOfTestSamples)));
	
	Status.printMessage("Mean X neg = " + String((float)(sumXneg) / (float)(numberOfTestSamples)) +
						"\tMean Y neg = " + String((sumYneg) / (float)(numberOfTestSamples)) +
						"\tMean Z neg = " + String((sumZneg) / (float)(numberOfTestSamples)));
	
	Status.printMessage("Sum X = " + String(sumXpos - sumXneg) + "\tSum Y = " + String(sumYpos - sumYneg) + "\tSum Z = " + String(sumZpos - sumZneg));
	
	Status.printMessage("Mean X = " + String((float)(sumXpos - sumXneg) / (2 * (float)(numberOfTestSamples))) + 
						"\tMean Y = " + String((sumYpos - sumYneg) / (2 * (float)(numberOfTestSamples))) + 
						"\tMean Z = " + String((sumZpos - sumZneg) / (2 * (float)(numberOfTestSamples))));
#endif

	/*
	*	CALCULATE GAIN AND OFFSET FACTOR:
	*
	*	According to datasheet page 19 - SELF TEST OPERATION,
	*	and formulas for gain and offset from from: http://www.analog.com/media/en/technical-documentation/application-notes/AN-1057.pdf	formulas - (17), (18) 
	*
	*	Gain = (Positive Samples - Negative samples) / [(Quantity of Positive Samples + Quantity of Negative Samples) * Self Test Axis Parameter * LSB per Gauss (corresponding to chosen gain)]
	*
	*	Offset = (Sum of Positive Samples + Sum of Negative samples) / (Quantity of Positive Samples + Quantity of Negative Samples)
	*/
	
#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
	Status.printMessage("Calculating gain factors:\n\tX\tY\tZ");
#endif

	this->gainFactor.X = (sumXpos - sumXneg) / (HMC5883L_SF_X_PARAMETER * 2 * (float)(numberOfTestSamples) * (float)this->getGainLSBperGauss(selfTestResolution));
	this->gainFactor.Y = (sumYpos - sumYneg) / (HMC5883L_SF_Y_PARAMETER * 2 * (float)(numberOfTestSamples) * (float)this->getGainLSBperGauss(selfTestResolution));
	this->gainFactor.Z = (sumZpos - sumZneg) / (HMC5883L_SF_Z_PARAMETER * 2 * (float)(numberOfTestSamples) * (float)this->getGainLSBperGauss(selfTestResolution));

#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
	Status.printMessage(String(this->gainFactor.X, 5) + "\t" + String(this->gainFactor.Y, 5) + "\t" + String(this->gainFactor.Z, 5));
#endif

#if(HMC5883L_CALIBRATION_DEBUG == 0x01)
	Status.printMessage("Calculating offset factors:\n\tX\tY\tZ");
#endif

	this->offsetFactor.X = 0.5F * (((float)(sumXpos) / (float)(numberOfTestSamples)) + ((float)(sumXneg) / (float)(numberOfTestSamples)));
	this->offsetFactor.Y = 0.5F * (((float)(sumYpos) / (float)(numberOfTestSamples)) + ((float)(sumYneg) / (float)(numberOfTestSamples)));
	this->offsetFactor.Z = 0.5F * (((float)(sumZpos) / (float)(numberOfTestSamples)) + ((float)(sumZneg) / (float)(numberOfTestSamples)));

#if(HMC5883L_CALIBRATION_DEBUG == 0x01)		
	Status.printMessage(String(this->offsetFactor.X, 5) + "\t" + String(this->offsetFactor.Y, 5) + "\t" + String(this->offsetFactor.Z, 5));
#endif
}

void HMC5883L::useSelfTestCompensationParameters(void)
{
	this->rawData.X = (((float)this->rawData.X - this->offsetFactor.X) / this->gainFactor.X);
	this->rawData.Y = (((float)this->rawData.Y - this->offsetFactor.Y) / this->gainFactor.Y);
	this->rawData.Z = (((float)this->rawData.Z - this->offsetFactor.Z) / this->gainFactor.Z);
}

void HMC5883L::useHardAndSoftIronCompensationParameters(void)
{
	const float ellipsoidCenter[3] = {-218.263, -61.0854, -16.4482};
	const float ellipsoidTransMatrix[3][3] = {	{0.831151, 0.00489485, 0.043762}, 
												{0.00489485, 0.833013, -0.000299692}, 
												{0.043762, -0.000299692, 0.988652}	};

	// Apply bias compensation - hard iron errors:
	sFloatAxes temp;
	temp.X = this->scaledData.X - ellipsoidCenter[0];
	temp.Y = this->scaledData.Y - ellipsoidCenter[1];
	temp.Z = this->scaledData.Z - ellipsoidCenter[2];

	// Apply transformation matrix compensation - soft iron errors:
	this->scaledData.X = temp.X * ellipsoidTransMatrix[0][0] + temp.Y * ellipsoidTransMatrix[0][1] + temp.Z * ellipsoidTransMatrix[0][2];
	this->scaledData.Y = temp.X * ellipsoidTransMatrix[1][0] + temp.Y * ellipsoidTransMatrix[1][1] + temp.Z * ellipsoidTransMatrix[1][2];
	this->scaledData.Z = temp.X * ellipsoidTransMatrix[2][0] + temp.Y * ellipsoidTransMatrix[2][1] + temp.Z * ellipsoidTransMatrix[2][2];
}

uint8_t HMC5883L::readStatusRegister(void)
{
	/*
	Register Bitfields
		[7:2] SR7:SR2 - reserved
				 
		[1]  - Data output register lock. This bit is set when:
				1.some but not all for of the six data output registers have been read,
				2. Mode register has been read.
				When this bit is set, the six data output registers are locked and any new data will not be placed in these register until one of these conditions are met: 
					2.1. all six bytes have been read, 
					2.2. the mode register is changed,
				3. the measurement configuration (CRA) is changed,
				4. power is reset
				
		[0] - Ready Bit. Set when data is written to all six data registers. 
			Cleared when device initiates a write to the data output registers 
			and after one or more of the data output registers are written to. 
			When RDY bit is clear it shall remain cleared for a 250 ?s. DRDY pin 
			can be used as an alternative to the status register for monitoring 
			the device for measurement data.
	*/
	
	return this->readByte(this->_I2CAddress, HMC5883L_STATUS);
}

eHMC5883LBitStatus HMC5883L::checkReadyBit(void)
{
	eHMC5883LBitStatus bitStatus = CLEAR;
	if(0x01 == (this->readStatusRegister() & 0x01))
	{
		bitStatus = SET;
	}

	return bitStatus;
}

eHMC5883LBitStatus HMC5883L::checkLockBit(void)
{
	eHMC5883LBitStatus bitStatus = CLEAR;
	if(0x02 == (this->readStatusRegister() & 0x02))
	{
		bitStatus = SET;
	}

	return bitStatus;
}

//--------------------------------------------------------------------
//-- HMC5883L ISR glue routines
//--------------------------------------------------------------------
/*
	According to:
	https://www.gammon.com.au/forum/?id=12983
	and
	http://forum.arduino.cc/index.php?topic=160101.0
	
	These are functions that interface between an ISR and an instance of a class.
*/

void HMC5883L::isrA(void)
{
	instanceA->handleInterrupt();
	
}

#if (HMC5883L_ENABLE_MORE_INSTANCES == 0x01)
void HMC5883L::isrB()
{
	instanceB->handleInterrupt();
}
#endif

void HMC5883L::handleInterrupt(void)
{
	this->dataReadyFlag = true;
}

//--------------------------------------------------------------------
//-- HMC5883L for use by ISR glue routines
//--------------------------------------------------------------------
HMC5883L * HMC5883L::instanceA = NULL;

#if (HMC5883L_ENABLE_MORE_INSTANCES == 0x01)
HMC5883L * HMC5883L::instanceB = NULL;
#endif

//--------------------------------------------------------------------
//-- HMC5883L EXTERN OBJECT
//--------------------------------------------------------------------
HMC5883L Magnetometer("HMC5883L", HMC5883L_ADDRESS, HMC5883L_A_DATA_READY_PIN);