#ifndef i2c_SI7021_h
#define i2c_SI7021_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the SI7021-Sensor

CONSUMPTION: sleep 0.6µA, measure 120-180µA
MEASUREMENT: 12ms for 12bit RH, 11ms for 14bit Temp
ACCURACY     in Main Range 0.4°C, 3% RH

With every humiditymeasurement there is an automatical temperaturmeasurement (just read it afterwards)

########################################################################  */

class SI7021 : public i2cSensor
{

    /** ######### Register-Map ################################################################# */

#define P_ADDRESS 	        0x60


    /** ######### function definition ################################################################# */

public:

    SI7021(void)
    {
        //_address = MPL_ADDRESS;
    };

    /**<  gives values */
    void getValue(uint8_t buffer[])
    {

    };

};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif




