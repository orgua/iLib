#ifndef i2c_preset_h
#define i2c_preset_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the preset-Sensor

 CONSUMPTION: standby X µA, measure X µA

 Details:


########################################################################  */

class PRESET : public i2cSensor
{

    /** ######### Register-Map ################################################################# */

#define P_ADDRESS 	        0x60


    /** ######### function definition ################################################################# */

public:
    /**< TODO: do i need a constructor? */
    PRESET(void)
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




