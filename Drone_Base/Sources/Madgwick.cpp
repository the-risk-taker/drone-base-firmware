/* 
* Madgwick.cpp
*
* Created: 27.06.2018 20:59:07
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*
* This work is based on:
* ---May 1, 2014, Kris Winer, GY-80 Basic Example Code, https://github.com/kriswiner/GY-80
*/

//--------------------------------------------------------------------
//-- MADGWICK INCLUDES
//--------------------------------------------------------------------
#include "../Headers/Madgwick.h"

//--------------------------------------------------------------------
//-- MADGWICK CLASS
//--------------------------------------------------------------------
Madgwick::Madgwick(float gyroMeasError, float gyroMeasDrift)
{
	this->madgwickBeta = (sqrt(3.0F / 4.0F) * gyroMeasError);
	this->madgwickZeta = (sqrt(3.0F / 4.0F) * gyroMeasDrift);
	
	this->setDeclinationAngle(CRACOW_DECLINATION_ANGLE);
}

/*
	Implementation of Sebastian Madgwick's "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
	(see http://www.x-io.co.uk/category/open-source/ for examples and more details).
*/
void Madgwick::quaternionUpdate(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0F * q1;
	float _2q2 = 2.0F * q2;
	float _2q3 = 2.0F * q3;
	float _2q4 = 2.0F * q4;
	float _2q1q3 = 2.0F * q1 * q3;
	float _2q3q4 = 2.0F * q3 * q4;
	
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(accX * accX + accY * accY + accZ * accZ);
	if(norm == 0.0) { return; }	// handle NaN
	norm = 1.0F/norm;			// use reciprocal for division
	accX *= norm;
	accY *= norm;
	accZ *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(magX * magX + magY * magY + magZ * magZ);
	if(norm == 0.0) { return; }	// handle NaN
	norm = 1.0F/norm;			// use reciprocal for division
	magX *= norm;
	magY *= norm;
	magZ *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0F * q1 * magX;
	_2q1my = 2.0F * q1 * magY;
	_2q1mz = 2.0F * q1 * magZ;
	_2q2mx = 2.0F * q2 * magX;
	hx = magX * q1q1 - _2q1my * q4 + _2q1mz * q3 + magX * q2q2 + _2q2 * magY * q3 + _2q2 * magZ * q4 - magX * q3q3 - magX * q4q4;
	hy = _2q1mx * q4 + magY * q1q1 - _2q1mz * q2 + _2q2mx * q3 - magY * q2q2 + magY * q3q3 + _2q3 * magZ * q4 - magY * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + magZ * q1q1 + _2q2mx * q4 - magZ * q2q2 + _2q3 * magY * q4 - magZ * q3q3 + magZ * q4q4;
	_4bx = 2.0F * _2bx;
	_4bz = 2.0F * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0F * q2q4 - _2q1q3 - accX) + _2q2 * (2.0F * q1q2 + _2q3q4 - accY) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - magX) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - magY) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - magZ);
	s2 = _2q4 * (2.0F * q2q4 - _2q1q3 - accX) + _2q1 * (2.0F * q1q2 + _2q3q4 - accY) - 4.0F * q2 * (1.0F - 2.0F * q2q2 - 2.0F * q3q3 - accZ) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - magX) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - magY) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - magZ);
	s3 = -_2q1 * (2.0F * q2q4 - _2q1q3 - accX) + _2q4 * (2.0F * q1q2 + _2q3q4 - accY) - 4.0F * q3 * (1.0F - 2.0F * q2q2 - 2.0F * q3q3 - accZ) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - magX) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - magY) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - magZ);
	s4 = _2q2 * (2.0F * q2q4 - _2q1q3 - accX) + _2q3 * (2.0F * q1q2 + _2q3q4 - accY) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - magX) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - magY) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - magZ);
	
	// Normalise step magnitude
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    
	norm = 1.0F/norm;			// use reciprocal for division
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q2 * gyroX - q3 * gyroY - q4 * gyroZ) - this->madgwickBeta * s1;
	qDot2 = 0.5f * (q1 * gyroX + q3 * gyroZ - q4 * gyroY) - this->madgwickBeta * s2;
	qDot3 = 0.5f * (q1 * gyroY - q2 * gyroZ + q4 * gyroX) - this->madgwickBeta * s3;
	qDot4 = 0.5f * (q1 * gyroZ + q2 * gyroY - q3 * gyroX) - this->madgwickBeta * s4;

	// Calculate integration time
	float timeDelta = this->getIntegrationTime();
	
	// Integrate to yield quaternion
	q1 += qDot1 * timeDelta;
	q2 += qDot2 * timeDelta;
	q3 += qDot3 * timeDelta;
	q4 += qDot4 * timeDelta;
	
	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    
	norm = 1.0F/norm;			// use reciprocal for division
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

/*
	Define output variables from updated quaternion - these are Tait-Bryan angles, commonly used in aircraft orientation.
	
	In this coordinate system, the positive z-axis is down toward Earth.
	--- Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, 
	looking down on the sensor positive yaw is counterclockwise.
	--- Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	--- Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	
	These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	applied in the correct order which for this configuration is yaw, pitch, and then roll.
	
	For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
*/
sEulerAngles Madgwick::getEulerAngles()
{
	sEulerAngles outputEulerAngles;
	
	outputEulerAngles.yaw   = atan2(2.0F * (this->q[1] * this->q[2] + this->q[0] * this->q[3]), this->q[0] * this->q[0] + this->q[1] * this->q[1] - this->q[2] * this->q[2] - this->q[3] * this->q[3]);
	outputEulerAngles.pitch = -asin(2.0F * (this->q[1] * this->q[3] - this->q[0] * this->q[2]));
	outputEulerAngles.roll  = atan2(2.0F * (this->q[0] * this->q[1] + this->q[2] * this->q[3]), this->q[0] * this->q[0] - this->q[1] * this->q[1] - this->q[2] * this->q[2] + this->q[3] * this->q[3]);
	
	outputEulerAngles.pitch *= 180.0F / PI;
	outputEulerAngles.yaw   *= 180.0F / PI;
	outputEulerAngles.yaw   -= this->declinationAngle;
	outputEulerAngles.roll  *= 180.0F / PI;
	
	return outputEulerAngles;
}

void Madgwick::setBeta(float beta)
{
	this->madgwickBeta = beta;
}

void Madgwick::setZeta(float zeta)
{
	this->madgwickZeta = zeta;
}

float Madgwick::getIntegrationTime(void)
{
	// Calculate integration time in [s]
	uint32_t			actualTime = micros();
	static uint32_t		lastTime = 0;
	
	float	timeDelta = ((actualTime - lastTime)/1000000.0F);
			lastTime = actualTime;
	
	return timeDelta;
}

void Madgwick::setDeclinationAngle(float angle)
{
	this->declinationAngle = angle;
}

//--------------------------------------------------------------------
//-- MADGWICK EXTERN OBJECT
//--------------------------------------------------------------------
Madgwick MadgwickFilter(MADGWICK_GYRO_MEAS_ERROR, MADGWICK_GYRO_MEAS_DRIFT);