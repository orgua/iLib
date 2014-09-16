#ifndef WirePlus_h
#define WirePlus_h

#include <Wire.h>
#include <Arduino.h> // for uint8_t data type


#define getmax(a,b) ((a)>(b)?(a):(b))
#define BITMASK(a)  (1<<a)
#define BIT(a)      (1<<a)

#ifndef TRUE
#define TRUE        (1==1)
#define FALSE       (1==2)
#endif
#define LOW         (0)
#define HIGH        (1)

class WirePlus
{

public:

    WirePlus();

    uint8_t probe(const uint8_t);
    uint8_t probeAddress(const uint8_t);
    void    writeByte(const uint8_t, const uint8_t, const uint8_t);
    void    writeCMD(const uint8_t, const uint8_t);
    uint8_t readByte(const uint8_t , const uint8_t );
    void    read(const uint8_t , const uint8_t , uint8_t *, const uint8_t );
    void    setRegister(const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    uint8_t getRegister(const uint8_t, const uint8_t, const uint8_t);

};

extern WirePlus i2c;

#endif
