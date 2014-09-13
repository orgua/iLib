
//#include "Wire.h"
#include <Wire.h>
#include "i2c.h"

// Initialize Class Variables //////////////////////////////////////////////////

//uint8_t WirePlus::rxBufferIndex = 0;

// Constructors ////////////////////////////////////////////////////////////////

WirePlus::WirePlus()
{
	Wire.begin(); 		// I2C as Master
	bitSet(PORTC, 4); 	// deactivate internal pull-ups for twi
	bitSet(PORTC, 5); 	// as per note from atmega8 manual pg167
	// switch to 400KHz I2C - eheheh
	TWBR = ((F_CPU / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
}

// Public Methods //////////////////////////////////////////////////////////////

uint8_t WirePlus::probe(uint8_t address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission(true)==0) 	{ return 1;}
	else 								{ return 0;}
}

uint8_t WirePlus::probeAddress(uint8_t address) {
    return probe(address);
}

void WirePlus::writeByte(uint8_t address, uint8_t register_address, uint8_t write_value)
{
	Wire.beginTransmission(address);
	Wire.write(register_address);
	Wire.write(write_value);
	Wire.endTransmission(true);
}

uint8_t WirePlus::readByte(uint8_t address, uint8_t register_address)
{
	uint8_t _readvalue;
	read(address, register_address, &_readvalue, 1);
	return _readvalue;
}


void WirePlus::read(uint8_t address, uint8_t registeraddress, uint8_t buff[], uint8_t size)
{
	Wire.beginTransmission(address); 	// Adress + WRITE (0)
	Wire.write(registeraddress);
	Wire.endTransmission(false); 		// No Stop Condition, for repeated Talk
	if (size) {
		Wire.requestFrom(address, size); 	// Address + READ (1)

		uint8_t _i; _i=0;

		//while (Wire.available() < size);
		while(Wire.available())
		{
		buff[_i] = Wire.read();
		_i++;
		}
		Wire.endTransmission(true); 		// Stop Condition
	}
}


void WirePlus::setRegister(uint8_t address, uint8_t registeraddress, uint8_t mask, uint8_t writevalue)
{
	uint8_t _setting;
	read(address, registeraddress, &_setting, 1 );
	writevalue	&= mask;
	_setting     &= ~mask;
	_setting     |= writevalue;
	writeByte(address, registeraddress, _setting);
}

uint8_t WirePlus::getRegister(uint8_t address, uint8_t registeraddress, uint8_t mask)
{
	uint8_t _setting;
	read(address, registeraddress, &_setting, (uint8_t)1 );
	return (_setting & mask);
}

// Preinstantiate Objects //////////////////////////////////////////////////////
WirePlus i2c = WirePlus();



