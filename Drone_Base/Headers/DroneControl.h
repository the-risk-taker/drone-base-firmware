/* 
* DroneControl.h
*
* Created: 23.07.2018 11:58:13
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#ifndef __DRONECONTROL_H__
#define __DRONECONTROL_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../Configs/Config.h"

class DroneControl
{
public:
	DroneControl(String _name);
	
	void begin();
	void init();
	
	String getName();
	
	void setWantedThrottle(float value);
	void setWantedPitch(float value);
	void setWantedRoll(float value);
	void setWantedYaw(float value);
	
	void setActualPitch(float newPitch);
	void setActualRoll(float newRoll);
	void setActualYaw(float newYaw);
	
	void setPitchOffset(float offset);
	void setRollOffset(float offset);
	
	void updatePIDs();
	void updateMotorControllers();
	void runMotors();
	void stopMotors();
	
	float getActualPitch();
	float getActualRoll();
	float getActualYaw();
	
	void setPitchPIDenabled(bool value);
	void setRollPIDenabled(bool value);
	void setOffsetEnabled(bool value);
	
	void setMotorOffset(uint8_t motorID, uint16_t offsetValue);
	
private:
	const String name;
	
	uint8_t movingAverageBufferSize;
	
	float	wantedThrottle,
			wantedPitch,
			wantedRoll,
			wantedYaw;
			
	float	pitchBuffer[ORIENTATION_MOVING_AVERAGE_BUFFER_SIZE],
			rollBuffer[ORIENTATION_MOVING_AVERAGE_BUFFER_SIZE],
			yawBuffer[ORIENTATION_MOVING_AVERAGE_BUFFER_SIZE];
			
	float	pitchPIDoutput,
			rollPIDoutput;
			
	float	actualPitch,
			actualRoll,
			actualYaw;
			
	float	pitchOffset,
			rollOffset;
			
	bool	pitchPIDenabled,
			rollPIDenabled,
			offsetEnabled;
			
	float	frontRightOffset,
			frontLeftOffset,
			rearRightOffset,
			rearLeftOffset;
};

extern DroneControl Drone;

#endif //__DRONECONTROL_H__
