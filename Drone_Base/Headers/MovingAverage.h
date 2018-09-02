/*
* MovingAverage.h
*
* Created: 22.07.2018 19:21:28
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/


#ifndef MOVINGAVERAGE_H_
#define MOVINGAVERAGE_H_

void moveElements(float *buffer, uint8_t lastElementIndex)
{
	for(uint8_t i = lastElementIndex; i > 0; i--)
	{
		buffer[i] = buffer[i - 1];
	}
}

float movingAverage(float *buffer, float newValue, uint8_t bufferSize)
{
	float sum = 0;
	
	buffer[0] = newValue;
	
	for(uint8_t i = 0; i < bufferSize; i++)
	{
		sum += buffer[i];
	}
	
	moveElements(buffer, (bufferSize - 1));

	return (sum/bufferSize);
}

#endif /* MOVINGAVERAGE_H_ */