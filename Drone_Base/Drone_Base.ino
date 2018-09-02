/*
* Drone_Base.ino
*
* Created: 6/3/2018 5:49:53 PM
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Release to public domain under MIT License, unless it states otherwise. Check files notes.
*/

#include <Wire.h>

#include "Headers/Status.h"
#include "Headers/I2C_Sensor.h"
#include "Headers/ADXL345.h"
#include "Headers/L3G4200D.h"
#include "Headers/BMP085.h"
#include "Headers/HMC5883L.h"
#include "Headers/Mahony.h"
#include "Headers/Madgwick.h"
#include "Headers/ProtocolAPI.h"
#include "Configs/Config.h"
#include "Headers/DroneMotorController.h"
#include "Headers/NRFLite.h"
#include "Headers/DueTimer.h"
#include "Headers/DroneControl.h"
#include "Headers/ComplementaryFilter.h"

ProtocolAPI *SerialProtocolAPI = new ProtocolAPI("Serial Communication", Serial, SHORT_RANGE_COMMUNICATION_SERIAL_BAUDRATE);
ProtocolAPI *BluetoothProtocolAPI = new ProtocolAPI("Bluetooth Communication", Serial3, SHORT_RANGE_COMMUNICATION_BLUETOOTH_BAUDRATE);
ProtocolAPI LongRangeComm("Long Range Communication", NATIVE_INTERFACE);

volatile bool timer1Flag = false;
bool watchdogStatus;

void setup()
{
	Wire.begin();
	
	Status.begin();

	Timer1.attachInterrupt(timer1Handler).setFrequency(DRONE_CONTROL_LOOP_FREQUENCY).start();
	
	pinMode(LED_BUILTIN, OUTPUT);
	toogleTestLed(25);
	
	Drone.begin();
	Drone.setPitchOffset(PITCH_OFFSET_VALUE);
	Drone.setRollOffset(ROLL_OFFSET_VALUE);

	SerialProtocolAPI->begin();
	BluetoothProtocolAPI->begin();
	LongRangeComm.begin();
	
	SerialUSB.begin(115200);
	SerialUSB.println("SerialUSB begins!");
	
	//TODO check on osciloscope
	//disableInternalI2CPullUp();
	
	Status.checkStatus(Accelerometer);
	Status.checkStatus(Gyroscope);
	Status.checkStatus(Barometer);
	Status.checkStatus(Magnetometer);
		
	watchdogEnable(DUE_WATCHDOG_TIMEOUT);
	watchdogStatus = watchdogCheckStatusRegister();
}

void loop()
{
	toogleTestLed(25);
	
	if(timer1Flag)
	{
		useAccelerometer();
		useGyroscope();
		useBarometer();
		useMagnetometer();
		
		useFilters();
	
		if(millis() > 3000) // Give time for filters to initialize.
		{
			Drone.setWantedPitch(0.0F);
			Drone.setWantedRoll(0.0F);
			
			sEulerAngles angles = MadgwickFilter.getEulerAngles();
			#if( TEST_COMPLEMENTARY_VS_MAHONY_MADGWICK_FILTERS == 0x01)
			sOrientation orientation = ComplementaryFilter(Accelerometer.getPitchRoll(), Gyroscope.getTemporaryTraveledAngle());
			
			String message =	"FPitch = " + String(angles.pitch) +
			"\tFRoll = " + String(angles.roll) +
			"\tCPitch = " + String(orientation.pitch) +
			"\tCRoll = " + String(orientation.roll);
			Serial.println(message);
			#endif
			
			Drone.setActualPitch(angles.pitch);
			Drone.setActualRoll(angles.roll);
			
			Drone.updatePIDs();
			Drone.updateMotorControllers();

			if(LongRangeComm.checkConnectionStatus())
			{
				Drone.runMotors();
			}
			else
			{
				Drone.stopMotors();
			}
		}

		timer1Flag = false;
	}
	
	LongRangeComm.protocolPackets->protocolBytes->readBytes();
	
	processBytes();				// by default every 5ms
	
	watchdogReset();
}

void useAccelerometer()
{
	Accelerometer.read();
	//Status.printMessage("X = " + String(Accelerometer.getRaw().X) + "\t Y = " + String(Accelerometer.getRaw().Y) + "\tZ = " + String(Accelerometer.getRaw().Z));
}

void useGyroscope()
{
	Gyroscope.read();
	//Status.printMessage("X = " + String(Gyroscope.getScaled().X) + "\tY = " + String(Gyroscope.getScaled().Y) + "\tZ = " + String(Gyroscope.getScaled().Z));
}

void useBarometer()
{
	Barometer.read();
	//Status.printMessage("Pressure\t" + String(Barometer.getPressure()) + "\tAltitude\t" + String(Barometer.getAltitude()) + "\tTemperature\t" + String(Barometer.getTemperature()));
}

void useMagnetometer()
{
	if(Magnetometer.getDataReadyFlag())
	{
		Magnetometer.read();
		//Status.printMessage("X = " + String(Magnetometer.getScaled().X) + "\tY = " + String(Magnetometer.getScaled().Y) + "\tZ = " + String(Magnetometer.getScaled().Z));
	}
}

void useFilters()
{				
	MadgwickFilter.quaternionUpdate(Accelerometer.getScaled().X, Accelerometer.getScaled().Y, Accelerometer.getScaled().Z,
									Gyroscope.getScaled().X * (PI / 180.0F), Gyroscope.getScaled().Y * (PI / 180.0F), Gyroscope.getScaled().Z * (PI / 180.0F),
									Magnetometer.getScaled().X, Magnetometer.getScaled().Y, Magnetometer.getScaled().Z);

	MahonyFilter.quaternionUpdate(Accelerometer.getScaled().X, Accelerometer.getScaled().Y, Accelerometer.getScaled().Z,
					Gyroscope.getScaled().X * (PI / 180.0F), Gyroscope.getScaled().Y * (PI / 180.0F), Gyroscope.getScaled().Z * (PI / 180.0F),
					Magnetometer.getScaled().X, Magnetometer.getScaled().Y, Magnetometer.getScaled().Z);
	
	//TODO check if is it worth
	//changeMadgwickFreeParameters();
}

void disableInternalI2CPullUp(void)
{
	digitalWrite(SDA, LOW);
	digitalWrite(SCL, LOW);
}

void changeMadgwickFreeParameters()
{
	 if(millis() > 10000) // 10 [s]
	 {
		 MadgwickFilter.setBeta(0.4F);
	 }
}

void processBytes()
{
	BluetoothProtocolAPI->protocolPackets->processInputBytes();
	BluetoothProtocolAPI->processPacket();
	SerialProtocolAPI->protocolPackets->processInputBytes();
	SerialProtocolAPI->processPacket();
	LongRangeComm.protocolPackets->processInputBytes();
	LongRangeComm.processPacket();
}

void serialEvent() // Serial
{
	SerialProtocolAPI->protocolPackets->protocolBytes->readBytes();
}

void serialEvent3() // BT
{
	BluetoothProtocolAPI->protocolPackets->protocolBytes->readBytes();
}

void watchdogSetup(){} // has to be overriden to enable watchdog
	
bool watchdogCheckStatusRegister()
{
	/*
	From Atmel datasheet SAM3X / SAM3A Series - Watchdog Timer Status Register
	
	Bitfields:
		WDUNF: Watchdog Underflow
			0: No Watchdog underflow occurred since the last read of WDT_SR.
			1: At least one Watchdog underflow occurred since the last read of WDT_SR.
		WDERR: Watchdog Error
			0: No Watchdog error occurred since the last read of WDT_SR.
			1: At least one Watchdog error occurred since the last read of WDT_SR.
	*/
	
	bool returnedStatus = false;
	
	uint32_t status = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> 8;	// Get status from the last Reset
	
	if(status == 0b10)
	{
		// Should be 0b010 after first watchdog reset
		Status.printErrorMesage("RSTTYP = 0b" +  String(status, BIN) + "\t Reset CPU occured!");	
		
		returnedStatus = true;
	}
	
	return returnedStatus;
}	

void toogleTestLed(uint32_t interval)
{
	static uint32_t prev = millis();
	static bool ledState = false;
	
	if((millis() - prev) >= interval)
	{
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState);
		prev = millis();
	}
}

void timer1Handler()
{
	timer1Flag = true;
}