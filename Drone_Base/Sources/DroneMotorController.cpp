/* 
* DroneMotorController.cpp
*
* Created: 17.07.2018 13:07:27
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#include "../Headers/DroneMotorController.h"
#include "../Configs/Config.h"

 DroneMotorController::DroneMotorController(String name, uint8_t motroControllerPWMPin, uint8_t id, eRotationOperation rotationOperation) :
	_name(name),
	_motorControllerPWMPin(motroControllerPWMPin),
	_motorID(id),
	_rotationOperation(rotationOperation)
{
	this->lockFlag = true;
	this->motorSpeed = ESC_MIN_PWM_VALUE;
}

bool DroneMotorController::begin()
{
	pinMode(this->_motorControllerPWMPin, OUTPUT);
	
	if(!this->initMotorController())
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool DroneMotorController::initMotorController()
{
	SerialUSB.println(MotorController.attach(_motorControllerPWMPin));
	
	if(MotorController.attach(_motorControllerPWMPin))
	{
		this->lockFlag = false;
		MotorController.writeMicroseconds(this->motorSpeed);
		return true;
	}
	else
	{
		SerialUSB.println(this->_name + String(" attach error!"));
		return false;
	}
}

void DroneMotorController::setSpeed(uint16_t speed)
{
	speed += ESC_MIN_PWM_VALUE;
	
	if(speed <= ESC_MIN_PWM_VALUE)
	{
		this->motorSpeed = ESC_MIN_PWM_VALUE;
	}
	else if(speed > ESC_MAX_PWM_VALUE)
	{
		this->motorSpeed = ESC_MAX_PWM_VALUE;
	}
	else
	{
		this->motorSpeed = speed;
	}
}

uint16_t DroneMotorController::getSpeed()
{
	return this->motorSpeed;
}

String DroneMotorController::getName()
{
	return this->_name;
}

uint8_t DroneMotorController::getMotorControllerPin()
{
	return this->_motorControllerPWMPin;
}

uint8_t DroneMotorController::getMotorID()
{
	return this->_motorID;
}

eRotationOperation DroneMotorController::getRotationOperation()
{
	return this->_rotationOperation;
}

void DroneMotorController::runMotor()
{
	if(!this->lockFlag)
	{
		if(this->motorSpeed <= ESC_MIN_PWM_VALUE)
		{
			this->MotorController.writeMicroseconds(ESC_MIN_PWM_VALUE);
		}
		else if(this->motorSpeed > ESC_MAX_PWM_VALUE)
		{
			this->MotorController.writeMicroseconds(ESC_MAX_PWM_VALUE);
		}
		else
		{
			this->MotorController.writeMicroseconds(this->motorSpeed);
		}
	}
	else
	{
		Serial.println(this->_name + " is locked!");
	}
}

Servo Dummy; // To prevent from returning attach method 0 value. Returning 0 locks the given controller (lockFlag is set).

DroneMotorController FrontRightMotorController(	"Front Right ESC", 
												FRONT_RIGHT_ESC_PWM_PIN,
												FRONT_RIGHT_MOTOR_ID_LABEL,
												COUNTER_CLOCKWISE);		// Mount color = BLUE		// id = 1

DroneMotorController FrontLeftMotorController(	"Front Left ESC", 
												FRONT_LEFT_ESC_PWM_PIN, 
												FRONT_LEFT_MOTOR_ID_LABEL,
												CLOCKWISE);					// Mount color = ORANGE		// id = 4

DroneMotorController RearRightMotorController(	"Rear Right ESC", 
												REAR_RIGHT_ESC_PWM_PIN,
												REAR_RIGHT_MOTOR_ID_LABEL,
												CLOCKWISE);			// Mount color = ORANGE		// id = 2

DroneMotorController RearLeftMotorController(	"Rear Left ESC",
												REAR_LEFT_ESC_PWM_PIN,
												REAR_LEFT_MOTOR_ID_LABEL,
												COUNTER_CLOCKWISE);		// Mount color = BLUE		// id = 3