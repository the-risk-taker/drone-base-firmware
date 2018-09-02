/* 
* DroneControl.cpp
*
* Created: 23.07.2018 11:58:12
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#include "../Headers/DroneControl.h"
#include "../Headers/MovingAverage.h"
#include "../Headers/DroneMotorController.h"
#include "../Headers/PID.h"

DroneControl::DroneControl(String _name) : name(_name)
{
	this->movingAverageBufferSize = ORIENTATION_MOVING_AVERAGE_BUFFER_SIZE;
	
	this->pitchPIDenabled = true;
	this->rollPIDenabled = true;
}

void DroneControl::begin()
{
	//SerialUSB.println(this->name + " started!");
	
	FrontLeftMotorController.begin();
	RearRightMotorController.begin();
	RearLeftMotorController.begin();
	FrontRightMotorController.begin();
}

void DroneControl::init()
{
	this->pitchOffset = 0.0F;
	this->rollOffset = 0.0F;
}

String DroneControl::getName()
{
	return this->name;
}

void DroneControl::setWantedThrottle(float value)
{
	this->wantedThrottle = value;
}

void DroneControl::setWantedPitch(float value)
{
	this->wantedPitch = value;
}

void DroneControl::setWantedRoll(float value)
{
	this->wantedRoll = value;
}

void DroneControl::setWantedYaw(float value)
{
	this->wantedYaw = value;
}

void DroneControl::setActualPitch(float newPitch)
{
	if(this->movingAverageBufferSize > 1)
	{
		this->actualPitch = movingAverage(this->pitchBuffer, newPitch, this->movingAverageBufferSize) - this->pitchOffset;
	}
	else
	{
		this->actualPitch = newPitch - this->pitchOffset;
	}
}

void DroneControl::setActualRoll(float newRoll)
{
	if(this->movingAverageBufferSize > 1)
	{
		this->actualRoll = movingAverage(this->rollBuffer, newRoll, this->movingAverageBufferSize) - this->rollOffset;
	}
	else
	{
		this->actualRoll = newRoll - this->rollOffset;
	}
}

void DroneControl::setActualYaw(float newYaw)
{
	if(this->movingAverageBufferSize > 1)
	{
		this->actualYaw = movingAverage(this->yawBuffer, newYaw, this->movingAverageBufferSize);
	}
	else
	{
		this->actualYaw = newYaw;
	}
}

void DroneControl::setPitchOffset(float offset)
{
	this->pitchOffset = offset;
}

void DroneControl::setRollOffset(float offset)
{
	this->rollOffset = offset;
}

void DroneControl::updatePIDs()
{
	this->pitchPIDoutput = PitchPID.update(this->wantedPitch - this->actualPitch, this->actualPitch);
	this->rollPIDoutput = RollPID.update(this->wantedRoll - this->actualRoll, this->actualRoll);
}

void DroneControl::updateMotorControllers()
{
	/*
		Motor speed:
		* speed = f(throttle, pitch, roll, yaw?);
		* speed = constrain{min, max}; 
	*/
	
	float	frontRightMotorSpeed = 0.0,
			frontLeftMotorSpeed = 0.0,
			rearRightMotorSpeed = 0.0,
			rearLeftMotorSpeed = 0.0;
	
	if(!this->pitchPIDenabled)
	{
		this->pitchPIDoutput = 0;
	}
	
	if(!this->rollPIDenabled)
	{
		this->rollPIDoutput = 0;
	}
	
	if(this->wantedThrottle > THROTTLE_VALUE_WHEN_PID_IS_STARTING_TO_WORK)
	{
		frontRightMotorSpeed = this->wantedThrottle - this->pitchPIDoutput + this->rollPIDoutput;
		frontLeftMotorSpeed = this->wantedThrottle - this->pitchPIDoutput - this->rollPIDoutput;
		rearRightMotorSpeed = this->wantedThrottle + this->pitchPIDoutput + this->rollPIDoutput;
		rearLeftMotorSpeed = this->wantedThrottle + this->pitchPIDoutput - this->rollPIDoutput;
	}
	else
	{
		frontRightMotorSpeed = this->wantedThrottle;
		frontLeftMotorSpeed = this->wantedThrottle;
		rearRightMotorSpeed = this->wantedThrottle;
		rearLeftMotorSpeed = this->wantedThrottle;
	}
	
	if(this->offsetEnabled && this->wantedThrottle > THROTTLE_VALUE_WHEN_PID_IS_STARTING_TO_WORK)
	{
		frontRightMotorSpeed += frontRightOffset;
		frontLeftMotorSpeed += frontLeftOffset;
		rearRightMotorSpeed += rearRightOffset;
		rearLeftMotorSpeed += rearLeftOffset;
	}
	
	FrontRightMotorController.setSpeed(frontRightMotorSpeed);
	FrontLeftMotorController.setSpeed(frontLeftMotorSpeed);
	RearRightMotorController.setSpeed(rearRightMotorSpeed);
	RearLeftMotorController.setSpeed(rearLeftMotorSpeed);
	
	#define TEST 0x01
	
#if( TEST == 0x01)
	String message =
	String(millis()) + "\t" +
	String(FrontRightMotorController.getSpeed()) + "\t" +
	String(FrontLeftMotorController.getSpeed()) + "\t" +
	String(RearRightMotorController.getSpeed()) + "\t" +
	String(RearLeftMotorController.getSpeed()) + "\t" +
	
	String(this->actualPitch) + "\t" +
	String(this->pitchPIDoutput) + "\t" +
	
	String(this->actualRoll) + "\t" +
	String(this->rollPIDoutput) + "\t" +
	
	String(this->wantedThrottle);
	
	Serial.println(message);
#endif
}

void DroneControl::runMotors()
{
	FrontRightMotorController.runMotor();
	FrontLeftMotorController.runMotor();
	RearRightMotorController.runMotor();
	RearLeftMotorController.runMotor();
}

void DroneControl::stopMotors()
{
	FrontRightMotorController.setSpeed(0);
	FrontLeftMotorController.setSpeed(0);
	RearRightMotorController.setSpeed(0);
	RearLeftMotorController.setSpeed(0);
	
	this->runMotors();
}


float DroneControl::getActualPitch()
{
	return this->actualPitch;
}

float DroneControl::getActualRoll()
{
	return this->actualRoll;
}

float DroneControl::getActualYaw()
{
	return this->actualYaw;
}

void DroneControl::setPitchPIDenabled(bool value)
{
	this->pitchPIDenabled = value;
}

void DroneControl::setRollPIDenabled(bool value)
{
	this->rollPIDenabled = value;
}

void DroneControl::setOffsetEnabled(bool value)
{
	this->offsetEnabled = value;
}

void DroneControl::setMotorOffset(uint8_t motorID, uint16_t offsetValue)
{
	#define FRONT_RIGHT_MOTOR   0
	#define FRONT_LEFT_MOTOR    1
	#define REAR_RIGHT_MOTOR    2
	#define REAR_LEFT_MOTOR     3
	
	switch(motorID)
	{
		case FRONT_RIGHT_MOTOR:
		{
			this->frontRightOffset = offsetValue;
		}
		break;
		case FRONT_LEFT_MOTOR:
		{
			this->frontLeftOffset = offsetValue;
		}
		break;
		case REAR_RIGHT_MOTOR:
		{
			this->rearRightOffset = offsetValue;
		}
		break;
		case REAR_LEFT_MOTOR:
		{
			this->rearLeftOffset = offsetValue;
		}
		break;
	}
}

DroneControl Drone("My drone");