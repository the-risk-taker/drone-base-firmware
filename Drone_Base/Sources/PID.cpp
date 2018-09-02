/* 
* PID.cpp
*
* Created: 21.07.2018 17:39:55
* Authors: Jakub Dabros, Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Jakub Dabros, Mateusz Patyk. All rights reserved.
*/

#include "../Headers/PID.h"

PID::PID(	String _name,
			float _integralState,
			float _derivativeState,
			float _integralMax,
			float _integralMin,
			float _proportionalGain,
			float _integralGain,
			float _derivativeGain
) :
			name(_name),
			integralState(_integralState),
			lastPosition(_derivativeState),
			integralMax(_integralMax),
			integralMin(_integralMin),
			proportionalGain(_proportionalGain),
			integralGain(_integralGain),
			derivativeGain(_derivativeGain)
{
}

float PID::update(float _error, float _newPosition)
{
	// NOTE:  sample time (dt) is included in hardware - timers and interrupts,
	// so there is no need to set tunnings Kp, Ki, Kd
	
	// CALCULATE THE PROPORTIONAL TERM
	this->pTerm = this->proportionalGain * _error;
	// CALCULATE THE INTEGRAL TERM WITH APPROPRIATE LIMITING
	// NOTE:  Ki * error is to eliminate undesirable bump while Ki changing

	this->integralState += _error;
	
	if(this->integralState > this->integralMax)
	{
		this->integralState = this->integralMax;
	}
	else if(this->integralState < this->integralMin)
	{
		this->integralState = this->integralMin;
	}
	this->iTerm = this->integralGain * this->integralState;	
	
	// CALCULATE THE DERIVATIVE TERM
	
	// NOTE:  dError(t)/dt = dWantedPosition(t)/dt - dCurrentPosition(t)/dt,
	//      when WantedPosition is constant: dError(t)/dt = -dCurrentPositon(t)/dt
	//      so dCurrentPositon(t)/dt ~ currentPosition - lastPosition (look at the minus!)
	
	this->dTerm = this->derivativeGain * (this->lastPosition - _newPosition);	
	
	this->lastPosition = _newPosition;
	
	// CALCULATE THE PID OUTPUT
	this->output = this->pTerm + this->dTerm + this->iTerm;
	
	return this->output;
}

void PID::setIntegralGain(float _gain)
{
	this->integralGain = _gain;
}

void PID::setProportionalGain(float _gain)
{
	this->proportionalGain = _gain;
}

void PID::setDerivativeGain(float _gain)
{
	this->derivativeGain = _gain;
}

String PID::getName()
{
	return this->name;
}

PID PitchPID(	"Pitch PID",
				PID_INTEGRAL_STATE_VALUE,
				PID_DERIVATE_STATE_VALUE,
				PID_INTEGRAL_MAX_VALUE,
				PID_INTEGRAL_MIN_VALUE,
				PID_PROPORTIONAL_GAIN_VALUE,
				PID_INTEGRAL_GAIN_VALUE,
				PID_DERIVATIVE_GAIN_VALUE);

PID RollPID(	"Roll PID",
				PID_INTEGRAL_STATE_VALUE,
				PID_DERIVATE_STATE_VALUE,
				PID_INTEGRAL_MAX_VALUE,
				PID_INTEGRAL_MIN_VALUE,
				PID_PROPORTIONAL_GAIN_VALUE,
				PID_INTEGRAL_GAIN_VALUE,
				PID_DERIVATIVE_GAIN_VALUE);