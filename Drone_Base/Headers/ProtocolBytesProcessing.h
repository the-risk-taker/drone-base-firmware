/* 
* Protocol.h
*
* Created: 07.07.2018 19:21:51
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#ifndef __PROTOCOL_BYTES_H__
#define __PROTOCOL_BYTES_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//--------------------------------------------------------------------
//-- PROTOCOL CLASS DEBUG
//--------------------------------------------------------------------
#define PROTOCOL_BYTES_INPUT_DATA_SNIFF				0x00	// Enable = 0x01, Disable = 0x00
#define PROTOCOL_BYTES_OUTPUT_DATA_SNIFF			0x00	// Enable = 0x01, Disable = 0x00

//--------------------------------------------------------------------
//-- PROTOCOL CLASS DEFINES
//--------------------------------------------------------------------
#define PROTOCOL_BYTES_MAX_RX_BUFFER_LENGTH			128
#define PROTOCOL_BYTES_MAX_TX_BUFFER_LENGTH			PROTOCOL_BYTES_MAX_RX_BUFFER_LENGTH

//--------------------------------------------------------------------
//-- PROTOCOL CLASS ENUMS
//--------------------------------------------------------------------
typedef enum eBufferType
{
	RX_BUFFER,
	TX_BUFFER
} eBufferType;

typedef enum eConnectionStatus
{
	CONNECTED,
	DISCONNECTED
} eConnectionStatus;

typedef enum eCommInterfaceType
{
	NATIVE_INTERFACE,
	SERIAL_INTERFACE,
	OTHER_INTERFACE
} eCommInterfaceType;

//--------------------------------------------------------------------
//-- PROTOCOL CLASS CLASS
//--------------------------------------------------------------------
class ProtocolBytesProcessing
{
public:
	ProtocolBytesProcessing(const String protocolName, HardwareSerial& UARTObject, const uint32_t baudrate, const eCommInterfaceType interfaceType = SERIAL_INTERFACE);
	ProtocolBytesProcessing(const String protocolName, const eCommInterfaceType interfaceType = NATIVE_INTERFACE, HardwareSerial& UARTObject = Serial, const uint32_t baudrate = 0);
	
	// Main methods:
	void begin(void);
	bool readBytes();
	bool writeBytes();

	// Buffers and buffer's cursors methods:
	uint8_t getRxBufferCursor();
	uint8_t* getBuffer(eBufferType bufferType);
	void setTxBufferCursor(uint8_t value);
	void copyBytesOnTxBuffer(uint8_t* source, const uint8_t howMany);
	
	void setConnectionStatus(eConnectionStatus status);
	eConnectionStatus getConnectionFlag();
	
	uint32_t getBaudrate();
	String getProtocolName();
	eCommInterfaceType getInterfaceType();
	
	// Utilities:
#if(PROTOCOL_BYTES_INPUT_DATA_SNIFF == 0x01 || PROTOCOL_BYTES_OUTPUT_DATA_SNIFF == 0x01)
	void printBuffer(uint8_t* buffer, const uint8_t length);
#endif

private:
	const String _protocolName;
	HardwareSerial& _UARTObject;
	const uint32_t _baudrate;
	const eCommInterfaceType _interfaceType;
	
	eConnectionStatus connectionStatus;

	uint8_t rxBufferCursor;
	uint8_t rxBuffer[PROTOCOL_BYTES_MAX_RX_BUFFER_LENGTH];
	uint8_t txBufferCursor;
	uint8_t txBuffer[PROTOCOL_BYTES_MAX_TX_BUFFER_LENGTH];
	
	void serialFlush(void);	
};

//--------------------------------------------------------------------
//-- STATUSCLASS EXTERN OBJECT
//--------------------------------------------------------------------
//extern ProtocolBytesProcessing Protocol;

#endif //__PROTOCOL_BYTES_H__
