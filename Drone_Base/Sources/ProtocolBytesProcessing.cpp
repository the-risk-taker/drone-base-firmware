/* 
* Protocol.cpp
*
* Created: 07.07.2018 19:21:51
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

//--------------------------------------------------------------------
//-- PROTOCOL CLASS INCLUDES
//--------------------------------------------------------------------
#include "../Headers/ProtocolBytesProcessing.h"
#include "../Headers/NRFLite.h"
#include "../Configs/Config.h"

//--------------------------------------------------------------------
//-- PROTOCOL CLASS CLASS
//--------------------------------------------------------------------
ProtocolBytesProcessing::ProtocolBytesProcessing(const String protocolName, HardwareSerial& UARTObject, const uint32_t baudrate, const eCommInterfaceType interfaceType) : 
	_protocolName(protocolName),
	_UARTObject(UARTObject), 
	_baudrate(baudrate),
	_interfaceType(interfaceType)
{
	this->rxBufferCursor = 0;	
	this->txBufferCursor = 0;
	
	this->connectionStatus = DISCONNECTED;
};

 ProtocolBytesProcessing::ProtocolBytesProcessing(const String protocolName, const eCommInterfaceType interfaceType /*= NATIVE_INTERFACE*/, HardwareSerial& UARTObject /*= Serial*/, const uint32_t baudrate /*= 0*/) : 
	 _protocolName(protocolName),
	 _UARTObject(UARTObject),
	 _baudrate(baudrate),
	 _interfaceType(interfaceType)
{
	this->rxBufferCursor = 0;
	this->txBufferCursor = 0;
	
	this->connectionStatus = DISCONNECTED;
}

void ProtocolBytesProcessing::begin(void)
{
	if(_interfaceType == SERIAL_INTERFACE)
	{
		this->_UARTObject.begin(this->_baudrate);
		this->serialFlush();
	}
	else if (_interfaceType == NATIVE_INTERFACE)
	{
		LongRangeRadio.init(NRF24L01_DRONE_BASE_RADIO_IDENTIFIER, NRF24L01_CE_PIN, NRF24L01_CSN_PIN);
	}
}

void ProtocolBytesProcessing::serialFlush()
{
	while(this->_UARTObject.available() > 0)
	{
		this->_UARTObject.read();
	}
}

bool ProtocolBytesProcessing::readBytes()
{
	bool readStatus = true;
	
	if(_interfaceType == SERIAL_INTERFACE)
	{
		while (this->_UARTObject.available() > 0)
		{
			if(this->rxBufferCursor < PROTOCOL_BYTES_MAX_RX_BUFFER_LENGTH)
			{
				this->rxBuffer[this->rxBufferCursor++] = this->_UARTObject.read();
			}
			else
			{
				this->_UARTObject.read();
				readStatus = false;
				SerialUSB.println("Not good, data was lost. Make PROTOCOL_MAX_RX_BUFFER_LENGTH more bigger, or process bytes more frequently.");
			}
		}
	#if(PROTOCOL_BYTES_INPUT_DATA_SNIFF == 0x01)
		this->printBuffer(this->rxBuffer, this->rxBufferCursor);
	#endif		
	}
	else if(_interfaceType == NATIVE_INTERFACE)
	{
		if(LongRangeRadio.hasData())
		{
			this->rxBufferCursor = LongRangeRadio.hasData();
			LongRangeRadio.readData(&this->rxBuffer);
			
		#if(PROTOCOL_BYTES_INPUT_DATA_SNIFF == 0x01)
			this->printBuffer(this->rxBuffer, this->rxBufferCursor);
		#endif			
		}
	}

	return readStatus;
}

void ProtocolBytesProcessing::copyBytesOnTxBuffer(uint8_t* source, const uint8_t howMany)
{
	for(uint8_t i = 0; i < howMany; i++)
	{
		this->txBuffer[i] = source[i];
	}
	
	this->txBufferCursor = howMany;
}

void ProtocolBytesProcessing::setConnectionStatus(eConnectionStatus status)
{
	this->connectionStatus = status;
}

eConnectionStatus ProtocolBytesProcessing::getConnectionFlag()
{
	return connectionStatus;
}

uint32_t ProtocolBytesProcessing::getBaudrate()
{
	return this->_baudrate;
}

String ProtocolBytesProcessing::getProtocolName()
{
	return this->_protocolName;
}

eCommInterfaceType ProtocolBytesProcessing::getInterfaceType()
{
	return this->_interfaceType;
}

#if(PROTOCOL_BYTES_INPUT_DATA_SNIFF == 0x01 || PROTOCOL_BYTES_OUTPUT_DATA_SNIFF == 0x01)
void ProtocolBytesProcessing::printBuffer(uint8_t *buffer, const uint8_t length)
{
	for(uint8_t i = 0; i < length; i++)
	{
		SerialUSB.print(buffer[i]);
		if(i < length - 1)
		{
			SerialUSB.print(" | ");
		}
	}
	SerialUSB.println();
}
#endif

bool ProtocolBytesProcessing::writeBytes()
{
	bool writeStatus = true;

	if(this->_interfaceType == SERIAL_INTERFACE)
	{
		if(this->_UARTObject.write(this->txBuffer, this->txBufferCursor) != this->txBufferCursor)
		{
			writeStatus = false;
		}
	}
	else if(_interfaceType == NATIVE_INTERFACE)
	{
		// IMPORTAT: make sure that first argument of NRFLite::send() is coresponding to recipient!
		// In this case it is NRF24L01_DRONE_REMOTE_RADIO_IDENTIFIER;
		writeStatus = LongRangeRadio.send(NRF24L01_DRONE_REMOTE_RADIO_IDENTIFIER, &this->txBuffer, this->txBufferCursor, NRFLite::NO_ACK);
		
	#if(PROTOCOL_BYTES_OUTPUT_DATA_SNIFF == 0x01)
		this->printBuffer(this->rxBuffer, this->rxBufferCursor);
	#endif	
	}
	
	this->txBufferCursor = 0;
	
	return writeStatus;
}

uint8_t* ProtocolBytesProcessing::getBuffer(eBufferType bufferType)
{
	uint8_t *bufferPointer = NULL;
	
	switch(bufferType)
	{
		case RX_BUFFER:
		{
			bufferPointer = this->rxBuffer;
		}
		break;
		case TX_BUFFER:
		{
			bufferPointer = this->txBuffer;
		}
	}
	
	return bufferPointer;
}

uint8_t ProtocolBytesProcessing::getRxBufferCursor()
{
	uint8_t tempRxCounter = this->rxBufferCursor;
	this->rxBufferCursor = 0;
	return tempRxCounter;
}

void ProtocolBytesProcessing::setTxBufferCursor(uint8_t value)
{
	this->txBufferCursor = value;
}

//--------------------------------------------------------------------
//-- PROTOCOL CLASS EXTERN OBJECT
//--------------------------------------------------------------------
//ProtocolBytesProcessing Protocol(Serial, PROTOCOL_BYTES_SERIAL_BAUDRATE);