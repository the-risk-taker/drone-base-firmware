/* 
* PID.h
*
* Created: 21.07.2018 17:39:55
* Authors: Jakub Dabros, Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Jakub Dabros, Mateusz Patyk. All rights reserved.
*/


#ifndef __PID_H__
#define __PID_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../Configs/Config.h"

class PID
{
public:
	PID(String _name,
		float _integralState,
		float _derivativeState,
		float _integralMax,
		float _integralMin,
		float _proportionalGain,
		float _integralGain,
		float _derivativeGain
	);
	
	float update (float _error, float _fposition);
	
	void setIntegralGain(float _gain);
	void setProportionalGain(float _gain);
	void setDerivativeGain(float _gain);
	
	String getName();
	
private:
	const String name;
	float integralState;
	float lastPosition;
	float integralMax;
	float integralMin;
	float proportionalGain;
	float integralGain;
	float derivativeGain;
	
	float output;
	
	float	pTerm,
			iTerm,
			dTerm;
};

extern PID PitchPID;
extern PID RollPID;

#endif //__PID_H__
