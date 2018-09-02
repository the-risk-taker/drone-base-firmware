/* 
* ProtocolAPI.cpp
*
* Created: 09.07.2018 15:41:10
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#include "../Headers/ProtocolAPI.h"
#include "../Headers/DroneControl.h"
#include "../Headers/PID.h"

 ProtocolAPI::ProtocolAPI(const String protocolName, HardwareSerial& UARTObject, uint32_t baudrate, eCommInterfaceType interfaceType /*= SERIAL_INTERFACE*/)
{
	this->protocolPackets = new ProtocolPacketsProcessing(protocolName,UARTObject, baudrate, interfaceType);
}

 ProtocolAPI::ProtocolAPI(const String protocolName, eCommInterfaceType interfaceType /*= NATIVE_INTERFACE*/)
{
	this->protocolPackets = new ProtocolPacketsProcessing(protocolName, interfaceType);
}

void ProtocolAPI::begin()
{
	this->protocolPackets->protocolBytes->begin();
}

void ProtocolAPI::processPacket()
{
	if(this->protocolPackets->getNewPacketsNumber())
	{
		for(uint8_t i = 0; i < this->protocolPackets->getNewPacketsNumber(); i++)
		{
			sProtocolPacket incomingPacket = this->protocolPackets->getNewPacketsTable()[i];

			this->encodePacket(incomingPacket);
		}
		
		this->protocolPackets->resetNewPacketsNumber();
	}
}

bool ProtocolAPI::checkConnectionStatus()
{
	bool status = true;
	if((millis() - previousMessageTime) >= 20)
	{
		//Serial.println("Reset motors");
		status = false;
	}
	
	return status;
}

void ProtocolAPI::sendPacket(eCommandCodes command)
{
	sProtocolPacket outcomingPacket;
	
	outcomingPacket.commandCode = (uint8_t)command;
	outcomingPacket.length = 0;
	
	this->processOutcommingPacketToBytesAndSend(outcomingPacket);
}

eConnectionStatus ProtocolAPI::getConnectionStatus()
{
	return this->protocolPackets->protocolBytes->getConnectionFlag();
}

void ProtocolAPI::processOutcommingPacketToBytesAndSend(sProtocolPacket packet)
{
	if(this->getConnectionStatus() == CONNECTED)
	{
		this->protocolPackets->rewritePacketToBytes(packet);
		this->protocolPackets->protocolBytes->writeBytes();
	}
	else if((eCommandCodes)packet.commandCode == INIT_COMM)
	{
		this->protocolPackets->rewritePacketToBytes(packet);
		this->protocolPackets->protocolBytes->writeBytes();
	}
	else
	{
		Serial.println(String(millis()) + "\t" + "\tThere is no connection on " + this->protocolPackets->protocolBytes->getProtocolName() + " protocol!");
	}
}

void ProtocolAPI::encodePacket(sProtocolPacket packet)
{
	// TODO uncomment to print packet
	//this->protocolPackets->printPacket(packet);
	
	eCommandCodes code = (eCommandCodes)packet.commandCode;
	
	//Serial.print("Received [code] = ");
	
	switch(code)
	{
		case NO_CODE:
		{
			Serial.println("NO_CODE");
		}
		break;
		case ERROR_CODE:
		{
			Serial.println("ERROR");
		}
		break;
		case WARNING_CODE:
		{
			Serial.println("WARNING");
		}
		break;
		case INIT_COMM:
		{
			Serial.println("INIT_COMM");
			
			this->protocolPackets->protocolBytes->setConnectionStatus(CONNECTED);
			this->sendPacket(COMM_STARTED);
		}
		break;
		case STOP_COMM:
		{
			Serial.println("STOP_COMM");
		}
		break;
		case COMM_STARTED:
		{
			Serial.println("COMM_STARTED");
		}
		break;
		case COMM_ENDED:
		{
			Serial.println("COMM_ENDED");
		}
		break;
		case ACKNOWLEDGEMENT:
		{
			Serial.println("ACKNOWLEDGEMENT");
		}
		break;
		case JOYSTICK_DATA:
		{
			uint16_t throttle;
			memcpy(&throttle, packet.data, sizeof(uint16_t));
			previousMessageTime = millis();

			Drone.setWantedThrottle(throttle);
		}
		break;
		case PID_P_VALUE:
		{
			float value;
			memcpy(&value, packet.data, sizeof(value));
			
			PitchPID.setProportionalGain(value);
			RollPID.setProportionalGain(value);
			
			Serial.println("P = " + String(value));
		}
		break;
		case PID_I_VALUE:
		{
			float value;
			memcpy(&value, packet.data, sizeof(value));
			
			PitchPID.setIntegralGain(value);
			RollPID.setIntegralGain(value);
			
			Serial.println("I = " + String(value));
		}
		break;
		case PID_D_VALUE:
		{
			float value;
			memcpy(&value, packet.data, sizeof(value));
			
			PitchPID.setDerivativeGain(value);
			RollPID.setDerivativeGain(value);

			Serial.println("D = " + String(value));
		}
		break;
		case ENABLE_PITCH_PID:
		{
			uint8_t value;
			memcpy(&value, packet.data, sizeof(value));
			
			Serial.println("ENABLE_PITCH_PID = " + String(value));
			
			Drone.setPitchPIDenabled(value);
		}
		break;
		case ENABLE_ROLL_PID:
		{
			uint8_t value;
			memcpy(&value, packet.data, sizeof(value));
			
			Serial.println("ENABLE_ROLL_PID = " + String(value));
			
			Drone.setRollPIDenabled(value);
		}
		break;
		case ENABLE_OFFSET:
		{
			uint8_t value;
			memcpy(&value, packet.data, sizeof(value));
			
			Serial.println("ENABLE_OFFSET = " + String(value));
			
			Drone.setOffsetEnabled(value);
		}
		break;
		case CHANGE_MOTOR_OFFSET:
		{
			#define FRONT_RIGHT_MOTOR   0
			#define FRONT_LEFT_MOTOR    1
			#define REAR_RIGHT_MOTOR    2
			#define REAR_LEFT_MOTOR     3
			
			uint16_t motorID;
			uint16_t offset;
			memcpy(&motorID, packet.data, sizeof(motorID));
			memcpy(&offset, packet.data + sizeof(motorID), sizeof(offset));
			
			Serial.println("MotorID = " + String(motorID) + "\tOffset = " + String(offset));
			
			Drone.setMotorOffset((uint8_t)motorID, offset);
		}
		break;
		default:
		{
			Serial.println("\tERROR\tUnknown command code!");
		}
		break;
	}
}