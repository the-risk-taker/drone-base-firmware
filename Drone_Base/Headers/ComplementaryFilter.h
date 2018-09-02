/*
* ComplementaryFilter.h
*
* Created: 07.08.2018 13:47:00
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#ifndef _COMPLEMENTARYFILTER_h
#define _COMPLEMENTARYFILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

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

inline sOrientation ComplementaryFilter(sOrientation accelerometerOrientation, sFloatAxes temporaryTravelledAngle, float coefficient = 0.05F)
{
	/*
		UNDERSTANDIG GYRO DATA:
		
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
	
	static sOrientation orientation;
	
	orientation.pitch = (1.0F - coefficient) * (orientation.pitch - temporaryTravelledAngle.X) + coefficient * accelerometerOrientation.pitch;
	orientation.roll = (1.0F - coefficient) * (orientation.roll + temporaryTravelledAngle.Y) + coefficient * accelerometerOrientation.roll;
	
	return orientation;
}

#endif