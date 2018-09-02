/*
* crc16.h
*
* Created: 07.07.2018 19:03:14
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#ifndef CRC16_H_
#define CRC16_H_

inline uint16_t crc16update(const uint16_t inputCRC, const uint8_t inputData)
{
	uint16_t outputCRC = inputCRC ^ ((uint16_t)inputData << 8);
	outputCRC = ((outputCRC >> 4) * 77);
	return outputCRC;
}

#endif /* CRC16_H_ */