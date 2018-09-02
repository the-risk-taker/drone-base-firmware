/* 
* Madgwick.h
*
* Created: 27.06.2018 20:59:07
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*
* This work is based on:
* ---May 1, 2014, Kris Winer, GY-80 Basic Example Code, https://github.com/kriswiner/GY-80
*/

#ifndef __MADGWICK_H__
#define __MADGWICK_H__

//--------------------------------------------------------------------
//-- MADGWICK INCLUDES
//--------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- MADGWICK PROGRAM DEFINES
//--------------------------------------------------------------------
#define CRACOW_DECLINATION_ANGLE					5.18F			// http://www.magnetic-declination.com/#

//--------------------------------------------------------------------
//-- MADGWICK FREE PARAMETERS
//--------------------------------------------------------------------
/*
	There is a tradeoff in the *beta* parameter between *accuracy* and *response speed*.
	In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError 2.7 degrees/s)
	was found to give optimal accuracy. However, with this value, the LSM9SD0 response 
	time is about 10 seconds to a stable initial quaternion.
	By increasing beta (GyroMeasError) by about a factor of fifteen to (40.0 degrees/s), the response
	time constant is reduced to ~2 sec
*/
#define MADGWICK_GYRO_MEAS_ERROR					(PI * (10.0F / 180.0F))  // Gyroscope measurement error in rads/s (shown as 40 deg/s)
#define MADGWICK_GYRO_MEAS_DRIFT					(PI * (1.0F / 180.0F))   // Gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

/*
	Madgwick scheme free parameters:
		- beta (Madgwick::madgwickBeta) connected with Gyro Meas Error
		- zeta (Madgwick::madgwickZeta) connected with Gyro Meas Drift
		
	Check computing these parametrs in Magdwick class constructor.
	
	Zeta should be very small or zero.
*/

//--------------------------------------------------------------------
//-- MADGWICK STRUCTURES
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
//-- MADGWICK CLASS
//--------------------------------------------------------------------
class Madgwick
{
public:
	Madgwick(float proportionalGain, float integralGain);

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
	
	void setBeta(float beta);
	void setZeta(float zeta);
	
	void setDeclinationAngle(float angle);
	
private:
	
	float	// Table for quaternions:
			q[4] = {1.0F, 0.0F, 0.0F, 0.0F},

			// Free parameters:
			madgwickBeta,
			madgwickZeta,
			
			// Check value for your city at http://www.magnetic-declination.com/#
			declinationAngle;
			
	float getIntegrationTime(void);
};

//--------------------------------------------------------------------
//-- MADGWICK EXTERN OBJECT
//--------------------------------------------------------------------
extern Madgwick MadgwickFilter;

#endif //__MADGWICK_H__
