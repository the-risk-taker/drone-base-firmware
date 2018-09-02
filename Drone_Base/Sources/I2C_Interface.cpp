/*
* I2C_Interface.cpp
*
* Created: 31.05.2018 14:18:50
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
*/

#include "../Headers/I2C_Interface.h"

/*
IN:
	slaveAddress		- The 7-bit address of the device to transmit to
	command				- Value to send as a single byte
	
OUT:
	trasmissionStatus	- Transmission status info
*/
eI2CStatus I2C_Interface::writeCommand(uint8_t slaveAddress, uint8_t command)
{
	/* Begin a transmission to the I2C slave device with the given address. */
	Wire.beginTransmission(slaveAddress);
	/* Queue bytes for transmission with the write() function. */
	Wire.write(command);
	/* Transmit bytes by calling endTransmission(). And check if ended with success*/
	return ((eI2CStatus)Wire.endTransmission());
}

/*
IN:
	slaveAddress			- The 7-bit address of the device to transmit to
	slaveRegisterAddress	- Value to send as a single byte - slave register address
	data					- Value to send as a single byte - data
	
OUT:
	trasmissionStatus		- Transmission status info
*/
eI2CStatus I2C_Interface::writeByte(uint8_t slaveAddress, uint8_t slaveRegisterAddress, uint8_t data)
{
	Wire.beginTransmission(slaveAddress);
	/* Put slave register address in TX buffer. */	
	Wire.write(slaveRegisterAddress);    
	/* Put data in TX buffer. */   
	Wire.write(data);
	return ((eI2CStatus)Wire.endTransmission());
}

/*
IN:
	slaveAddress			- The 7-bit address of the device to transmit to
	slaveRegisterAddress	- Value to send as a single byte - slave register address
	
OUT:
	data					- Register data from slave.
	
NOTE: Here there is no checking if transmission was successful.
*/
uint8_t I2C_Interface::readByte(uint8_t slaveAddress, uint8_t slaveRegisterAddress)
{
	/* Data will store the register data. */
	uint8_t data; 
	
	Wire.beginTransmission(slaveAddress);
	Wire.write(slaveRegisterAddress);
	/* Send the Tx buffer, but send a restart (false argument) to keep connection alive. */
	Wire.endTransmission(false);             
	/* Request one byte from slave register address. */
	Wire.requestFrom(slaveAddress, (uint8_t) 1);
	/* Read byte from slave. */
	data = Wire.read();   
	                   
	return data;
}

/*
IN:
	slaveAddress			- The 7-bit address of the device to transmit to
	slaveRegisterAddress	- Value to send as a single byte - slave register address
	&destination			- Address of the element to store the data from slave
	
OUT:
	trasmissionStatus		- Transmission status info
*/
eI2CStatus I2C_Interface::readByte(uint8_t slaveAddress, uint8_t slaveRegisterAddress, uint8_t &destination)
{
	Wire.beginTransmission(slaveAddress);
	Wire.write(slaveRegisterAddress);
	/* Send the Tx buffer, but send a restart (false argument) to keep connection alive. */
	eI2CStatus transmissionStatus = (eI2CStatus)Wire.endTransmission(false);
	/* Request one byte from slave register address. */
	Wire.requestFrom(slaveAddress, (uint8_t) 1);
	/* Read byte from slave. */
	destination = Wire.read();
	
	return transmissionStatus;
}

/*
IN:
	slaveAddress			- The 7-bit address of the device to transmit to
	slaveRegisterAddress	- Value to send as a single byte - slave register address
	numbersOfBytesToRead	- Number of bytes written to slave
	*destination			- First element of data array which stores the data from slave
OUT:
	trasmissionStatus		- Transmission status info
*/
eI2CStatus I2C_Interface::readBytes(uint8_t slaveAddress, uint8_t slaveRegisterAddress, uint8_t numberOfBytesToRead, uint8_t *destination)
{
	Wire.beginTransmission(slaveAddress);
	Wire.write(slaveRegisterAddress); 
	/* Send the Tx buffer, but send a restart (false argument) to keep connection alive. */
	eI2CStatus transmissionStatus = (eI2CStatus)Wire.endTransmission(false);
	/* Request numbersOfBytesToRead bytes from slave register address. */
	Wire.requestFrom(slaveAddress, numberOfBytesToRead);
	
	uint8_t i = 0;
	while (Wire.available())
	{
		/* Read bytes from slave. */
		destination[i++] = Wire.read(); 
	}
	
	return transmissionStatus;
}