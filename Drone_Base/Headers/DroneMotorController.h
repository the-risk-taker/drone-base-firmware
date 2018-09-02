/* 
* DroneMotorController.h
*
* Created: 17.07.2018 13:07:27
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#ifndef __DRONEARM_H__
#define __DRONEARM_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef ROTATION_OPERATION
#define ROTATION_OPERATION
typedef enum eRotationOperation
{
	CLOCKWISE = 0,
	COUNTER_CLOCKWISE
} eRotationOperation;
#endif

#include "Servo.h"

class DroneMotorController
{
public:
	DroneMotorController(String name, uint8_t motroControllerPWMPin, uint8_t id, eRotationOperation rotationOperation);
	
	bool begin();
	bool initMotorController();
	
	void setSpeed(uint16_t speed);
	uint16_t getSpeed();
	String getName();
	uint8_t getMotorControllerPin();
	uint8_t getMotorID();
	eRotationOperation getRotationOperation();
	
	void runMotor();

private:
	Servo MotorController;

	const String _name;
	const uint8_t	_motorControllerPWMPin,
					_motorID;
	
	bool lockFlag;
	
	uint16_t	motorSpeed;
						
	eRotationOperation _rotationOperation;
};

extern DroneMotorController FrontRightMotorController;
extern DroneMotorController FrontLeftMotorController;
extern DroneMotorController RearRightMotorController;
extern DroneMotorController RearLeftMotorController;


#endif //__DRONEARM_H__
