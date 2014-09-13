#ifndef WirePlus_h
#define WirePlus_h

#include <Wire.h>
#include <Arduino.h> // for uint8_t data type

class WirePlus
{
	public:

	WirePlus();

	uint8_t probe(uint8_t);
	uint8_t probeAddress(uint8_t);
	void    writeByte(uint8_t, uint8_t, uint8_t);
	uint8_t readByte(uint8_t , uint8_t );
	void    read(uint8_t , uint8_t , uint8_t *, uint8_t );
	void    setRegister(uint8_t , uint8_t , uint8_t , uint8_t );
	uint8_t getRegister(uint8_t , uint8_t , uint8_t );

};

extern WirePlus i2c;

#endif
