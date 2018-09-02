/* 
* Mahony.cpp
*
* Created: 27.06.2018 20:09:06
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*
* This work is based on:
* ---May 1, 2014, Kris Winer, GY-80 Basic Example Code, https://github.com/kriswiner/GY-80
*/

//--------------------------------------------------------------------
//-- MAHONY INCLUDES
//--------------------------------------------------------------------
#include "../Headers/Mahony.h"

//--------------------------------------------------------------------
//-- MAHONY CLASS
//--------------------------------------------------------------------
Mahony::Mahony(float proportionalGain, float integralGain) : _proportionalGain(proportionalGain), _integralGain(integralGain)
{
	this->setDeclinationAngle(CRACOW_DECLINATION_ANGLE);
}

/*
	Implementation of Mahony Filer
	Similar to Madgwick scheme but uses proportional and integral filtering
	on the error between estimated reference vectors and measured ones.
*/
void Mahony::quaternionUpdate(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ)
{
	float q1 = this->q[0], q2 = this->q[1], q3 = this->q[2], q4 = this->q[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
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
	norm = 1.0F / norm;			// use reciprocal for division
	accX *= norm;
	accY *= norm;
	accZ *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(magX * magX + magY * magY + magZ * magZ);
	if(norm == 0.0) { return; }	// handle NaN
	norm = 1.0F / norm;			// use reciprocal for division
	magX *= norm;
	magY *= norm;
	magZ *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0F * magX * (0.5f - q3q3 - q4q4) + 2.0F * magY * (q2q3 - q1q4) + 2.0F * magZ * (q2q4 + q1q3);
	hy = 2.0F * magX * (q2q3 + q1q4) + 2.0F * magY * (0.5f - q2q2 - q4q4) + 2.0F * magZ * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0F * magX * (q2q4 - q1q3) + 2.0F * magY * (q3q4 + q1q2) + 2.0F * magZ * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0F * (q2q4 - q1q3);
	vy = 2.0F * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0F * bx * (0.5f - q3q3 - q4q4) + 2.0F * bz * (q2q4 - q1q3);
	wy = 2.0F * bx * (q2q3 - q1q4) + 2.0F * bz * (q1q2 + q3q4);
	wz = 2.0F * bx * (q1q3 + q2q4) + 2.0F * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (accY * vz - accZ * vy) + (magY * wz - magZ * wy);
	ey = (accZ * vx - accX * vz) + (magZ * wx - magX * wz);
	ez = (accX * vy - accY * vx) + (magX * wy - magY * wx);
	
	// Calculate and use integral feedback if enabled (greater than 0.0)
	if(this->_integralGain > 0.0F)
	{
		// Accumulate integral errors
		this->eInt[0] += ex;
		this->eInt[1] += ey;
		this->eInt[2] += ez;
	}
	else
	{
		// Prevent integral from windup
		this->eInt[0] = 0.0F;
		this->eInt[1] = 0.0F;
		this->eInt[2] = 0.0F;
	}

	// Apply feedback terms
	gyroX = gyroX + this->_proportionalGain * ex + this->_integralGain * this->eInt[0];
	gyroY = gyroY + this->_proportionalGain * ey + this->_integralGain * this->eInt[1];
	gyroZ = gyroZ + this->_proportionalGain * ez + this->_integralGain * this->eInt[2];

	// Calculate integration time
	float timeDelta = this->getIntegrationTime();
	
	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gyroX - q3 * gyroY - q4 * gyroZ) * (0.5f * timeDelta);
	q2 = pa + (q1 * gyroX + pb * gyroZ - pc * gyroY) * (0.5f * timeDelta);
	q3 = pb + (q1 * gyroY - pa * gyroZ + pc * gyroX) * (0.5f * timeDelta);
	q4 = pc + (q1 * gyroZ + pa * gyroY - pb * gyroX) * (0.5f * timeDelta);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0F / norm;			// use reciprocal for division
	this->q[0] = q1 * norm;
	this->q[1] = q2 * norm;
	this->q[2] = q3 * norm;
	this->q[3] = q4 * norm;
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
sEulerAngles Mahony::getEulerAngles()
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

void Mahony::setProportionalGain(float proportionalGain)
{
	this->_proportionalGain = proportionalGain;
}

void Mahony::setIntegralGain(float integralGain)
{
	this->_integralGain = integralGain;
}

float Mahony::getIntegrationTime(void)
{
	// Calculate integration time in [s]
	uint32_t			actualTime = micros();
	static uint32_t		lastTime = 0;
	
	float	timeDelta = ((actualTime - lastTime)/1000000.0F);
			lastTime = actualTime;
	
	return timeDelta;
}

void Mahony::setDeclinationAngle(float angle)
{
	this->declinationAngle = angle;
}

//--------------------------------------------------------------------
//-- MAHONY EXTERN OBJECT
//--------------------------------------------------------------------
Mahony MahonyFilter(MAHONY_KP_GAIN_DEFAULT, MAHONY_KI_GAIN_DEFAULT);