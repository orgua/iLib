#ifndef i2c_MAX44009_h
#define i2c_MAX44009_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the MAX44009-Sensor

CONSUMPTION: sleep ???, measure 0.65 - 1.6µA
MEAS_RANGE: 45 mLux - 180k Lux


########################################################################  */

class MAX44009 : public i2cSensor
{

    /** ######### Register-Map ################################################################# */

#define MAX_ADDRESS 				0x4B

#define INT_STATUS_REG				0x00
#define INT_ENABLE_REG              0x01
#define CONFIGURATION_REG           0x02
#define   CONFIG_CONT_MASK          (bit(7))  // CONTINOUS MODE
#define     CONFIG_CONT_ON          (1<<7)
#define     CONFIG_CONT_OFF         (0)
#define   CONFIG_MANUAL_MASK        (bit(6))  // MANUAL Set CDR and TIME
#define     CONFIG_MANUAL_ON        (1<<6)
#define     CONFIG_MANUAL_OFF       (0)
#define   CONFIG_CDR_MASK           (bit(3))  // Current DIVISION RATIO (When HIGH Brightness --> Current/8
#define     CONFIG_CDR_1div1        0
#define     CONFIG_CDR_1div8        (1<<3)
#define   CONFIG_TIM_MASK           (bit(2)|bit(1)|bit(0))
#define     CONFIG_TIM_800MS        (0)
#define     CONFIG_TIM_400MS        (1)
#define     CONFIG_TIM_200MS        (2)
#define     CONFIG_TIM_100MS        (3)
#define     CONFIG_TIM_50MS         (4)
#define     CONFIG_TIM_25MS         (5)
#define     CONFIG_TIM_12MS         (6)
#define     CONFIG_TIM_6MS          (7)

#define LUX_HIGH_REG                0x03
#define LUX_LOW_REG                 0x04
#define THRESHOLD_UPPER_REG         0x05
#define THRESHOLD_LOWER_REG         0x06
#define THRESHOLD_TIMER_REG         0x07

    /** ######### function definition ################################################################# */

public:

    MAX44009(void)
    {
        //_address = MPL_ADDRESS;
    };


    void setEnabled(const uint8_t enable=1)
    {
        if (enable)
        {
            i2c.setRegister(MAX_ADDRESS, INT_ENABLE_REG, 1, 0);
            i2c.setRegister(MAX_ADDRESS, CONFIGURATION_REG, CONFIG_CONT_MASK, CONFIG_CONT_ON);
            i2c.setRegister(MAX_ADDRESS, CONFIGURATION_REG, CONFIG_MANUAL_MASK, CONFIG_MANUAL_OFF);
        }
        else
        {
            i2c.setRegister(MAX_ADDRESS, CONFIGURATION_REG, CONFIG_CONT_MASK, CONFIG_CONT_OFF);
        }
    };

    void reset()
    {
    // nothing to do
    };

    uint8_t initialize()
    {
        if (i2c.probe(MAX_ADDRESS))
        {
            setEnabled(1);
            return 1;
        }
        else
        {
            return 0;
        }
    };

    /**
     READING only high-register:
     Lux = 2(exponent) x mantissa x 0.72
     Exponent = 8xE3 + 4xE2 + 2xE1 + E0
     Mantissa = 8xM7 + 4xM6 + 2xM5 + M4

     READING combined Registers:
     E3–E0: Exponent bits of lux reading
     M7–M0: Mantissa byte of lux reading
     Lux = 2(exponent) x mantissa x 0.045
     */
    void getValue(uint32_t& mLux_value)
    {
        uint8_t lux[2], lux_exponent;

        i2c.read(MAX_ADDRESS, LUX_HIGH_REG, lux, 2);

        lux_exponent    = ((lux[0] >> 4) & 0x0F);
        lux[0]          = ((lux[0] << 4) & 0xF0);
        lux[1]         &= 0x0F;

        //lux_value    = 0.045 * ( lux_high + lux_low ) * (1<< lux_exponent);
        mLux_value    = 45L * ( lux[0] | lux[1] ) * (1<< lux_exponent);
    };




};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif




