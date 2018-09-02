/* 
* ProtocolAPI.h
*
* Created: 09.07.2018 15:41:10
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/


#ifndef __PROTOCOL_API_H__
#define __PROTOCOL_API_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../Headers/ProtocolPacketsProcessing.h"

//--------------------------------------------------------------------
//-- PROTOCOL API ENUMS
//-------------------------------------------------------------------
typedef enum
{
	NO_CODE,					// Restricted
	ERROR_CODE,					// Restricted
	WARNING_CODE,				// Restricted
	INIT_COMM,					// Restricted
	STOP_COMM,					// Restricted
	COMM_STARTED,				// Restricted
	COMM_ENDED,					// Restricted
	ACKNOWLEDGEMENT,			// Restricted

	JOYSTICK_DATA = 10,

	PID_P_VALUE = 20,
	PID_I_VALUE,
	PID_D_VALUE,

	PITCH_VALUE = 30,

	ENABLE_PITCH_PID = 40,
	ENABLE_ROLL_PID,
	ENABLE_OFFSET,
	
	CHANGE_MOTOR_OFFSET = 50

} eCommandCodes;

//--------------------------------------------------------------------
//-- PROTOCOL API CLASS
//-------------------------------------------------------------------
class ProtocolAPI
{
public:
	ProtocolAPI(const String protocolName, HardwareSerial& UARTObject, const uint32_t baudrate, const eCommInterfaceType interfaceType = SERIAL_INTERFACE);
	ProtocolAPI(const String protocolName, const eCommInterfaceType interfaceType = NATIVE_INTERFACE);
	
	ProtocolPacketsProcessing *protocolPackets;
	
	void begin();
	
	// Main methods, working constantly:
	void processPacket();
	bool checkConnectionStatus();
	
	// Main methods, working on events:
	void sendPacket(eCommandCodes command);
	template <typename T>
	void sendPacket(eCommandCodes command, T value);
	template <typename T>
	void sendPacket(eCommandCodes command, T* value, uint8_t howMany);
	
	eConnectionStatus getConnectionStatus();

private:
	uint32_t previousMessageTime;
	
	void processOutcommingPacketToBytesAndSend(sProtocolPacket packet);
	// Here write your project functionalities:
	void encodePacket(sProtocolPacket packet);
};

template <typename T>
void ProtocolAPI::sendPacket(eCommandCodes command, T value)
{
	sProtocolPacket outcomingPacket;
	
	outcomingPacket.commandCode = (uint8_t)command;
	outcomingPacket.length = sizeof(T);
	memcpy(outcomingPacket.data, &value, sizeof(T));
	
	this->processOutcommingPacketToBytesAndSend(outcomingPacket);
}

template <typename T>
void ProtocolAPI::sendPacket(eCommandCodes command, T* value, uint8_t howManyElements)
{
	sProtocolPacket outcomingPacket;
	
	outcomingPacket.commandCode = (uint8_t)command;
	outcomingPacket.length = sizeof(T)*howManyElements;
	
	if(outcomingPacket.length > PROTOCOL_PACKETS_MAX_DATA_LENGTH)
	{
		SerialUSB.println("You wanted to send too much data (" + String(outcomingPacket.length) + " bytes) in one packet (command code = " + String(outcomingPacket.commandCode) + "). Increase PROTOCOL_PACKETS_MAX_DATA_LENGTH or send two packets.");
		return;
	}
	
	for(uint8_t i = 0; i < howManyElements; i++)
	{
		memcpy(outcomingPacket.data + i * sizeof(T), (value + i), sizeof(T));
	}
	
	this->processOutcommingPacketToBytesAndSend(outcomingPacket);
}

#endif //__PROTOCOL_API_H__