/*
* L3G4200D.cpp
*
* Created: 04.06.2018 11:21:50
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/ 

//--------------------------------------------------------------------
//-- L3G4200D INCLUDES
//--------------------------------------------------------------------
#include "../Headers/L3G4200D.h"
#include "../Headers/Status.h"

//--------------------------------------------------------------------
//-- L3G4200D CLASS
//--------------------------------------------------------------------
 L3G4200D::L3G4200D(String sensorName, uint8_t I2CAddress) : _sensorName(sensorName), _I2CAddress(I2CAddress)
 {
 }
 
eBeginStatus L3G4200D::begin(void)
{
	eBeginStatus returnedStatus = BEGIN_ERROR;

	while(returnedStatus != BEGIN_COMPLETED)
	{
		if(0xD3 == this->getDeviceID())
		{
			returnedStatus = BEGIN_COMPLETED;
			
			this->init();
		}
	}

	return returnedStatus;
}

void L3G4200D::init(void)
{
	this->setDataRateAndBandwidth(DRBW_200_50); // Execute subesquently L3G4200D::setCTRL_REG1() to obtain effect
	this->setCTRL_REG2();  
	this->setCTRL_REG3();
	this->setScale(SCALE_250DPS);				// Execute subesquently L3G4200D::setCTRL_REG4() to obtain effect
	this->setCTRL_REG5();
	
	this->lowPassFilterSetting(ENABLE);
	this->calibrationSetting(ENABLE);
}

void L3G4200D::read(void)
{
	uint8_t rawData[NUMBER_OF_REGISTERS];
	
	/*
		IMPORTANT: To read the data sequentially, 7'th bit of the register address OUT_X_L
		must be set to 1, then you can request 6 bytes:
		
		(L3G4200D_OUT_X_L | L3G4200D_ENABLE_AUTOINCREMENT
	*/
	
	this->readBytes(this->_I2CAddress, (L3G4200D_OUT_X_L | L3G4200D_ENABLE_AUTOINCREMENT), NUMBER_OF_REGISTERS, &rawData[0]);
	
	this->_rawAngularVelocities.X = (((int16_t)rawData[X_HIGH_BYTE]) << 8) | rawData[X_LOW_BYTE];
	this->_rawAngularVelocities.Y = (((int16_t)rawData[Y_HIGH_BYTE]) << 8) | rawData[Y_LOW_BYTE];
	this->_rawAngularVelocities.Z = (((int16_t)rawData[Z_HIGH_BYTE]) << 8) | rawData[Z_LOW_BYTE];	
	if(ENABLE == this->_lowPassEnableFlag)
	{
		this->lowPassFilter(L3G4200D_LOW_PASS_FILTER_ALPHA);
	}
}

String L3G4200D::getDeviceName()
{
	return this->_sensorName;
}

uint8_t L3G4200D::getDeviceAddress()
{
	return this->_I2CAddress;
}

void L3G4200D::setDataRateAndBandwidth(eL3G4200D_DatarateAndBandwidth value)
{
	this->_datarateAndBandwidth = value;
	
	this->setCTRL_REG1();
}

eL3G4200D_DatarateAndBandwidth L3G4200D::getDataRateAndBandwidthFromDevice(void)
{
	return (eL3G4200D_DatarateAndBandwidth)(this->readByte(this->_I2CAddress, L3G4200D_CTRL_REG4) & L3G4200D_DTBW_MASK);
}

void L3G4200D::setScale(eL3G4200D_Scale scale)
{
	this->_scale = scale;
	
	/*
	From L3G4200D datasheet:
		[DPS]		[mdps/digit]	[dps/digit]
		FS = 250	8.75			0.00875
		FS = 500	17.50			0.01750
		FS = 2000	70				0.070
	*/
	
	switch(scale)
	{
		case SCALE_250DPS:
		{
			this->_resolution = 0.00875F;
		}
		break;
		case SCALE_500DPS:
		{
			this->_resolution = 0.01750F;
		}
		break;
		case SCALE_2000DPSa:
		{
			this->_resolution = 0.070F;
		}
		case SCALE_2000DPSb:
		{
			this->_resolution = 0.070F;
		}
		break;
	}
	
	this->setCTRL_REG4();
}

eL3G4200D_Scale L3G4200D::getScaleFromDevice(void)
{
	return (eL3G4200D_Scale)(this->readByte(this->_I2CAddress, L3G4200D_CTRL_REG1) & L3G4200D_SCALE_MASK);
}

void L3G4200D::readTemperature(void)
{
	/* 
		Calculate the temperature difference. This is not absolute temperature in C or F.
		
		The L3G4200D is provided with an internal temperature sensor that is suitable for delta
		temperature measurement. Temperature data are generated with a frequency of 1 Hz and
		are stored inside the OUT_TEMP register in two’s complement format, with a sensitivity if -1
		LSB/°C.
		
		IMPORTANT: 1Hz fresh rate! No need to read faster.
	*/
	this->_temperature = this->readByte(this->_I2CAddress, L3G4200D_OUT_TEMP);
}

uint8_t L3G4200D::getTemperature(void)
{
	return this->_temperature;
}

sIntAxes L3G4200D::getRaw(void)
{
	// Get raw angular velocity
	
	return this->_rawAngularVelocities;
}

sFloatAxes L3G4200D::getFiltered(void)
{
	if(DISABLE == this->_lowPassEnableFlag)
	{
		Status.printErrorMesage(this->getDeviceName() + "\tFiltering was disabled! This data is incorrect");
	}
	
	return this->_filteredAngularVelocities;
}

sFloatAxes L3G4200D::getScaled(void)
{
	/*
		Multiplying raw/(filtered raw) angular velocity by resolution we getting the scaled angular velocity in [1/s] = degree per second (dps).
		
			scaled angular velocity (?) = raw angular velocity from gyro * resolution
			? = raw_gyro * resolution
	*/
	
	if(ENABLE == this->_lowPassEnableFlag)
	{
		this->_angularScaledVelocities.X = this->_filteredAngularVelocities.X * this->_resolution;
		this->_angularScaledVelocities.Y = this->_filteredAngularVelocities.Y * this->_resolution;
		this->_angularScaledVelocities.Z = this->_filteredAngularVelocities.Z * this->_resolution;	
	}
	else
	{
		this->_angularScaledVelocities.X = this->_rawAngularVelocities.X * this->_resolution;
		this->_angularScaledVelocities.Y = this->_rawAngularVelocities.Y * this->_resolution;
		this->_angularScaledVelocities.Z = this->_rawAngularVelocities.Z * this->_resolution;
	}
	
	if(ENABLE == this->_calibrationEnableFlag)
	{
		this->_angularScaledVelocities.X = this->_angularScaledVelocities.X - L3G4200D_X_OFFSET;
		this->_angularScaledVelocities.Y = this->_angularScaledVelocities.Y - L3G4200D_Y_OFFSET;
		this->_angularScaledVelocities.Z = this->_angularScaledVelocities.Z - L3G4200D_Z_OFFSET;
	}
	
	return this->_angularScaledVelocities;
}

void L3G4200D::calculateTemporaryTraveledAngle()
{
	/*
		The road can be calculated by:
			S = V * t
		
		So the temporary angle traveled (a_temp) over a period of time (deltaT) 
		can be calculated by multiply the angular velocity (w) by this period of time (deltaT):
			a_temp = w * deltaT
		
		Going further, we can calculate the angle of rotation (pitch, roll, yaw)
		by integrating (summing) temporary angles:
			1)	integral(w)dt - in analog domain (integrating) 
			2)	sum(w * deltaT) - in digital domain (summing) 
					in practise:	actual_rotation = previous_rotation + actual_a_temp;
					in code:		rotation += angularScaledVelocity * deltaT
					
		This is important when complementary filter is needed
	*/
	
	static long previousTime = millis();
	
	float dt = (float)(millis() - previousTime)/1000.0F;
	previousTime = millis();
	
	// Integrate the gyroscope data -> integral(angularSpeed) = angle
	this->temporaryAngle.X = this->_angularScaledVelocities.X * dt;		// Angle around the X-axis
	this->temporaryAngle.Y = this->_angularScaledVelocities.Y * dt;		// Angle around the Y-axis
	this->temporaryAngle.Z = this->_angularScaledVelocities.Z * dt;		// Angle around the Z-axis
}

sFloatAxes L3G4200D::getTemporaryTraveledAngle(void)
{
	this->calculateTemporaryTraveledAngle();
	
	return this->temporaryAngle;
}

void L3G4200D::lowPassFilterSetting(eSetting flag)
{
	this->_lowPassEnableFlag = flag;
}

void L3G4200D::calibrationSetting(eSetting flag)
{
	this->_calibrationEnableFlag = flag;
}

void L3G4200D::performCalibrationProcedure(void)
{
	this->calibrationSetting(DISABLE);
	
	Status.printMessage("Calibration in: 3 seconds! DON'T MOVE DEVICE");
	delay(1000);
	Status.printMessage("Calibration in: 2 seconds!");
	delay(1000);
	Status.printMessage("Calibration in: 1 seconds!");
	delay(1000);
	
	sFloatAxes sum;
	sum.X = 0;
	sum.Y = 0;
	sum.Z = 0;
	
	uint16_t samples = 5000;
	for(uint16_t i = 0; i < samples; i++)
	{
		this->read();
		
		sum.X += this->getScaled().X;
		sum.Y += this->getScaled().Y;
		sum.Z += this->getScaled().Z;
		delay(1);
	}
	
	sum.X /= (float)samples;
	sum.Y /= (float)samples;
	sum.Z /= (float)samples;
	
	Status.printMessage("Xmean = " + String(sum.X, 5) + "\tYmean = " + String(sum.Y, 5) + "\tZmean = " + String(sum.Z, 5));
	Status.printMessage("In L3G4200D.h change define values\n\t\tL3G4200D_X_OFFSET to " + String(sum.X, 5) + "\n" + 
															 "\t\tL3G4200D_Y_OFFSET to " + String(sum.Y, 5) + "\n" + 
															 "\t\tL3G4200D_Z_OFFSET to " + String(sum.Z, 5)
						);
	Status.printMessage("Then uncomment Gyroscope.performCalibrationProcedure() in your program. Make sure that Gyroscope.calibrationSetting() is enabled");		
	Status.printMessage("While loop started!");
	while(1){};	
}

uint8_t L3G4200D::getDeviceID(void)
{
	uint8_t deviceID;
	
	// Should return 0xD3; 
	this->readByte(this->_I2CAddress, L3G4200D_REG_ID, deviceID);
	
	return deviceID;
}

void L3G4200D::lowPassFilter(float alpha)
{
	this->_filteredAngularVelocities.X = alpha * (float)this->_rawAngularVelocities.X + (1.0 - alpha) * this->_filteredAngularVelocities.X;
	this->_filteredAngularVelocities.Y = alpha * (float)this->_rawAngularVelocities.Y + (1.0 - alpha) * this->_filteredAngularVelocities.Y;
	this->_filteredAngularVelocities.Z = alpha * (float)this->_rawAngularVelocities.Z + (1.0 - alpha) * this->_filteredAngularVelocities.Z;
}

void L3G4200D::setCTRL_REG1(void)
{
	/*
	Register Bitfields
		[7:6] DR - Output data rate
		[5:4] BW - Bandwidth
		[3] PD - Power-down mode
		[2] ZEN - Z axis enable
		[1] YEN - Y axis enable
		[0] XEN - X axis enable
	*/
	
	uint8_t registerValue = L3G4200D_CLEAR_REGISTER;
	registerValue |= (this->_datarateAndBandwidth << 4);
	registerValue |= (L3G4200D_ENABLE_NORMAL_POWER_MODE << 3);
	registerValue |= L3G4200D_ENABLE_ALL_AXES;
	
	this->writeByte(this->_I2CAddress, L3G4200D_CTRL_REG1, registerValue);
}

void L3G4200D::setCTRL_REG2(void)
{
	/*
	Register Bitfields
		[7:6] Reserved
		[5:4] HPM1, HPM0 - High Pass filter Mode Selection, Default value: 00:
				00 Normal mode (reset reading HP_RESET_FILTER)
				01 Reference signal for filtering
				10 Normal mode
				11 Autoreset on interrupt event
		[3:0] HPCF3, HPCF2, HPCF1, HPCF0 - High Pass filter Cut Off frequency selection:
				HPCF3	ODR= 100 Hz ODR= 200 Hz ODR= 400 Hz ODR= 800 Hz
				0000	8			15			30			56
				0001	4			8			15			30
				0010	2			4			8			15
				0011	1			2			4			8
				0100	0.5			1			2			4
				0101	0.2			0.5			1			2
				0110	0.1			0.2			0.5			1
				0111	0.05		0.1			0.2			0.5
				1000	0.02		0.05		0.1			0.2
				1001	0.01		0.02		0.05		0.1
	*/
	
	// All defaults
	uint8_t registerValue = L3G4200D_CLEAR_REGISTER;
	//registerValue |= ...; // Set register to enable High Pass Filtering
	//registerValue &= 0x3F; // And make sure that the two MSB bits are 0.

	this->writeByte(this->_I2CAddress, L3G4200D_CTRL_REG2, registerValue);
}

void L3G4200D::setCTRL_REG3(void)
{
	/*
	Register Bitfields
		[7] I1_Int1		Interrupt enable on INT1 pin.				Default value: 0. (0: Disable; 1: Enable)
		[6] I1_Boot		Boot status available on INT1.				Default value: 0. (0: Disable; 1: Enable)
		[5] H_Lactive	Interrupt active configuration on INT1.		Default value: 0. (0: High; 1:Low)
		[4] PP_OD		Push- Pull / Open drain.					Default value: 0. (0: Push- Pull; 1: Open drain)
		[3] I2_DRDY		Date Ready on DRDY/INT2.					Default value: 0. (0: Disable; 1: Enable)
		[2] I2_WTM		FIFO Watermark interrupt on DRDY/INT2.		Default value: 0. (0: Disable; 1: Enable)
		[1] I2_ORun		FIFO Overrun interrupt on DRDY/INT2			Default value: 0. (0: Disable; 1: Enable)
		[0] I2_Empty	FIFO Empty interrupt on DRDY/INT2.			Default value: 0. (0: Disable; 1: Enable)
	*/
	
	uint8_t registerValue = L3G4200D_CLEAR_REGISTER;
	registerValue |= L3G4200D_ENABLE_DR_INTERRUPT;
	
	this->writeByte(this->_I2CAddress, L3G4200D_CTRL_REG3, registerValue);
}

void L3G4200D::setCTRL_REG4(void)
{
	/*
	Register Bitfields
		[7]		BDU			- Block Data Update.						Default value: 0	(0: continous update; 1: output registers not updated until MSB and LSB reading)
		[6]		BLE			- Big/Little Endian Data Selection.			Default value: 0.
		[5:4]	FS1 FS0		- Full Scale selection.						Default value: 00	(00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps) ST1-ST0 Self Test Enable. Default value: 00
					+/- 250DPS		-> 8.75 mDPS/LSB
					+/- 1000DPS		-> 17.5 mDPS/LSB
					+/- 2000DPS		-> 70.0 mDPS/LSB
		[3]		RESERVED	-
		[2:1]	ST1 ST0		- Self Test Enable.							Default value: 00	(00: Self Test Disabled;)
		[0]		SIM			- SIM SPI Serial Interface Mode selection.	Default value: 0	(0: 4-wire interface; 1: 3-wire interface).
	*/
	
	uint8_t registerValue = L3G4200D_CLEAR_REGISTER;
	registerValue |= L3G4200D_ENABLE_BDU;
	registerValue |= (this->_scale << 4);

	this->writeByte(this->_I2CAddress, L3G4200D_CTRL_REG4, registerValue);
}

void L3G4200D::setCTRL_REG5(void)
{
	/*
	Register Bitfields
		[7] BOOT						Reboot memory content.			Default value: 0 (0: normal mode; 1: reboot memory content)
		[6] FIFO_EN						FIFO_EN FIFO enable.			Default value: 0 (0: FIFO disable; 1: FIFO Enable)
		[5] RESERVED					-
		[4] HPen						HPen High Pass filter Enable.	Default value: 0 (0: HPF disabled; 1: HPF enabled. See Figure 20)
		[3:2] INT1_Sel1,	INT1_Sel0	INT1 selection configuration.	Default value: 0
		[1:0] Out_Sel1, Out_Sel0		Out selection configuration.	Default value: 0
	*/
	
	// BOOT - normal mode, FIFO disabled, High pass filter disabled, INT1, Out_Sel - default
	uint8_t registerValue = L3G4200D_CLEAR_REGISTER;
	
	this->writeByte(this->_I2CAddress, L3G4200D_CTRL_REG5, registerValue);
}

//--------------------------------------------------------------------
//-- L3G4200D EXTERN OBJECT
//--------------------------------------------------------------------
L3G4200D Gyroscope("L3G4200D", L3G4200D_ADDRESS);