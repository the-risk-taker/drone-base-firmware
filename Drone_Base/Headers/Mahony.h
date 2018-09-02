/* 
* Mahony.h
*
* Created: 27.06.2018 20:09:06
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*
* This work is based on:
* ---May 1, 2014, Kris Winer, GY-80 Basic Example Code, https://github.com/kriswiner/GY-80
*/

#ifndef __MAHONY_H__
#define __MAHONY_H__

//--------------------------------------------------------------------
//-- MAHONY INCLUDES
//--------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- MAHONY PROGRAM DEFINES
//--------------------------------------------------------------------
#define CRACOW_DECLINATION_ANGLE					5.18F			// http://www.magnetic-declination.com/#

//--------------------------------------------------------------------
//-- MAHONY FREE PARAMETERS
//--------------------------------------------------------------------
//----- Kp for proportional feedback, Ki for integral
#define MAHONY_KP_GAIN_DEFAULT						(2.0F * 5.0F)	// 2 * Kp
#define MAHONY_KI_GAIN_DEFAULT						(2.0F * 0.0F)	// 2 * Ki

//--------------------------------------------------------------------
//-- MAHONY STRUCTURES
//--------------------------------------------------------------------
#ifndef EULER_STRUCTURE
#define EULER_STRUCTURE
struct sEulerAngles
{
	float yaw;
	float pitch;
	float roll;
};
#endif

//--------------------------------------------------------------------
//-- MAHONY CLASS
//--------------------------------------------------------------------
class Mahony
{
public:
	Mahony(float proportionalGain, float integralGain);

	/*
		Accelerometer data:
		--- accX
		--- accY
		--- accZ
		should be provided in G [1g etc] values!
		
		Gyroscope data:
		--- gyroX
		--- gyroY
		--- gyroZ
		should be provided in rad [radians] values!
		
		Magnetometer data:
		--- magX
		--- magY
		--- magZ
		should be provided in mGs [mili Gauss] values!
	*/
	
	void quaternionUpdate(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ);
	sEulerAngles getEulerAngles();
	
	void setProportionalGain(float proportionalGain);	// Mahony Kp
	void setIntegralGain(float integralGain);			// Mahony Ki
	
	void setDeclinationAngle(float angle);
	
private:
			
	float	// Table for quaternions
			q[4] = {1.0F, 0.0F, 0.0F, 0.0F},
	
			// Table for integral error in Mahony method   
			eInt[3] = {0.0F, 0.0F, 0.0F},
	
			// Free parameters:
			_proportionalGain,
			_integralGain,
			
			// Check value for your city at http://www.magnetic-declination.com/#
			declinationAngle;
			
	float getIntegrationTime(void);
};

//--------------------------------------------------------------------
//-- MAHONY EXTERN OBJECT
//--------------------------------------------------------------------
extern Mahony MahonyFilter;

#endif //__MAHONY_H__