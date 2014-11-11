#ifndef i2c_LPS331_h
#define i2c_LPS331_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the LPS331-Sensor

 CONSUMPTION: standby X µA, measure X µA

 Details:

///
########################################################################  */

class LPS331 : public i2cSensor, public manualSensor
{

    /** ######### Register-Map ################################################################# */

#define LPS331AP_ADDRESS_SA0_LOW    0b1011100
#define LPS331AP_ADDRESS_SA0_HIGH   0b1011101
#define LPS_ADDRESS				    LPS331AP_ADDRESS_SA0_HIGH

#define LPS331_REF_P_XL             0x08
#define LPS331_REF_P_L              0x09
#define LPS331_REF_P_H              0x0A

#define LPS331_WHO_AM_I             0x0F
#define     VAL_WHO_AM_I            0xBB

#define LPS331_RES_CONF             0x10

#define LPS331_CTRL_REG1            0x20
#define LPS331_CTRL_REG2            0x21
#define LPS331_CTRL_REG3            0x22
#define LPS331_INTERRUPT_CFG        0x23
#define LPS331_INT_SOURCE           0x24
#define LPS331_THS_P_L              0x25
#define LPS331_THS_P_H              0x26
#define LPS331_STATUS_REG           0x27

#define LPS331_PRESS_OUT_XL         0x28
#define LPS331_PRESS_OUT_L          0x29
#define LPS331_PRESS_OUT_H          0x2A

#define LPS331_TEMP_OUT_L           0x2B
#define LPS331_TEMP_OUT_H           0x2C

#define LPS331_AMP_CTRL             0x30

#define LPS331_DELTA_PRESS_XL       0x3C
#define LPS331_DELTA_PRESS_L        0x3D
#define LPS331_DELTA_PRESS_H        0x3E

    /** ######### function definition ################################################################# */

public:
    /**< TODO: do i need a constructor? */
    LPS331(void)
    {
        //_address = MPL_ADDRESS;
    };

    void setDatarate(uint8_t valueHz)
    {
        if      (valueHz == 0)  valueHz = 0b00000000; // ONE-SHOT / MANUAL
        else if (valueHz == 1)  valueHz = 0b00010000;    // Pressure 512 averages, Temp 128 averages
        else if (valueHz <= 7)  valueHz = 0b01010000; // 7 Hz
        else if (valueHz <= 13) valueHz = 0b01100000; // 12.5 Hz
        else                    valueHz = 0b01110000; // 25Hz, Temp 64 averages
        i2c.setRegister(LPS_ADDRESS, LPS331_CTRL_REG1, B01110000, valueHz);	// chip active, 12.5Hz, no INT, continuous update, no delta_pressure, SPI/I2C
    };

    void setSensitivity(uint8_t sens=0)
    {
        if (sens) sens = 0x7A;
        else      sens = 0x6A;
        i2c.writeByte(LPS_ADDRESS, LPS331_RES_CONF, sens);	// temp 128samples, pres 512samples per Value
    }

    void setEnabled(uint8_t enable = 1)
    {
        if (enable) enable = 255;
        else        enable = 0;
        i2c.setRegister(LPS_ADDRESS, LPS331_CTRL_REG1, B10000000, enable);
    }

    void reset(void)
    {
        i2c.setRegister(LPS_ADDRESS, LPS331_CTRL_REG2, B10000100, B10000100); // Boot and SWRESET
    }


    /**< sets or detects slave address; returns bool indicating success */
    uint8_t initialize(void)
    {
        return initialize(25);
    };

    uint8_t initialize(uint8_t hzRate)
    {
        if (i2c.probe(LPS_ADDRESS)==0) return 0;

        i2c.writeByte(LPS_ADDRESS, LPS331_CTRL_REG1,	0b00000000);	// Chip power DOWN
        i2c.writeByte(LPS_ADDRESS, LPS331_CTRL_REG2,	0b00000000);   // NO auto_zero, NO one_shot

        setDatarate(hzRate);
        setSensitivity(0);
        setEnabled(1);

        return 1;
    };

    /**< only used when in manual/standby mode */
    void triggerMeasurement(void)
    {
        i2c.setRegister(LPS_ADDRESS, LPS331_CTRL_REG2, 0x01, 0x01);
    }

    /**< check for new data, return 1 when Measurement is ready */
    uint8_t checkMeasurement(void)
    {
        uint8_t _byte;
        i2c.read(LPS_ADDRESS,LPS331_STATUS_REG, &_byte, 1);
        if (_byte&0x02) return 1; // Pressure Measurement finished,
        else            return 0;
    };

    /**<  wait for new data*/
    uint8_t awaitMeasurement(void)
    {
        uint8_t _counter = 0;
        while (checkMeasurement()==0)
        {
            if (++_counter > 250) return 0; // check took longer than 250ms
            delay(1);
        };
        return 1; // Measurement finished
    };


    /**< reads pressure in millibars (mbar)/hectopascals (hPa) */
    void getMeasurement(float& mbar)
    {
        int32_t praw;
        getMeasurement(praw);
        mbar = (praw/4096.0);
    };

    /**< reads pressure and returns raw 24-bit sensor output */
    void inline getMeasurement(int32_t& pressure_raw)
    {
        uint8_t _byte[3];
        // assert MSB to enable register address auto-increment
        i2c.read(LPS_ADDRESS,LPS331_PRESS_OUT_XL | (1 << 7), _byte, 3);

        // combine bytes
        //  GCC performs an arithmetic right shift for signed negative
        //  numbers, but this code will not work if you port it to a
        //  compiler that does a logical right shift instead.
        pressure_raw = ((int32_t)_byte[2] << 16) | ((uint16_t)_byte[1] << 8) | (_byte[0]);
    };

    /**< reads temperature in degrees C */
    void getTemperature(float& degC)
    {
        int16_t _degRaw;
        getTemperature(_degRaw);
        degC = 42.5 + ((float)_degRaw) / 480.0;
    };

    /**< reads temperature and returns raw 16-bit sensor output */
    void getTemperature(int16_t& degRaw)
    {
        uint8_t _byte[2];
        // assert MSB to enable register address auto-increment
        i2c.read(LPS_ADDRESS,LPS331_TEMP_OUT_L | (1 << 7), _byte, 2);

        // combine bytes
        //  GCC performs an arithmetic right shift for signed negative
        //  numbers, but this code will not work if you port it to a
        //  compiler that does a logical right shift instead.
        degRaw = ((int16_t)_byte[1] << 8) | (_byte[0]);
    };



    /**< reads pressure and returns raw 24-bit sensor output */
    /*
    unsigned long LPS331::readAltitudeMillimeters(void)
    {
    uint8_t _byte[3];
    // assert MSB to enable register address auto-increment
    i2c.read(address,LPS331_PRESS_OUT_XL | (1 << 7), _byte, (byte) 3);

    unsigned long x;

    x = uint32_t(pow ((int32_t)_byte[2] << 16 | (uint16_t)_byte[1] << 8 | _byte[0] , 0.190263) * 2440790.3);
    return (44330800 - x);
    }
    */

    void inline getAltitude(float& height_m)
    {
        int32_t _cm;
        getAltitude(_cm);
        height_m = _cm / 100.0;
    }

    void inline getAltitude(int32_t& height_cm)
    {
        const long xnode[8]	= {2867200,		3101257,	3335314,	3569371,	3803429,	4037486,	4271543,	4505600};
        const long yfac[7]	= {1088,		1023,		967,		916,		872,		832,		796}; // * -4096
        const long ysum[7]	= {1062587,		1013703,	967631,		924033,		882630,		843188,		805511};

        unsigned long presraw;
        uint8_t _byte[3];
        i2c.read(LPS_ADDRESS,LPS331_PRESS_OUT_XL | (1 << 7), _byte, (byte) 3);
        presraw = ((int32_t)_byte[2] << 16 | (uint16_t)_byte[1] << 8 | _byte[0]);
        //presraw = readPressureRaw();

        byte n;
        if (presraw<xnode[4])
        {
            if (presraw<xnode[2])
            {
                if (presraw<xnode[1])
                {
                    if (presraw<=xnode[0])
                    {
                        n = 0;
                        presraw = xnode[0];
                    }
                    else  n = 0;
                }
                else n=1;
            }
            else
            {
                if (presraw<xnode[3]) n = 2;
                else                  n = 3;
            };
        }
        else
        {
            if (presraw < xnode[6])
            {
                if (presraw < xnode[5])  n = 4;
                else                     n = 5;
            }
            else
            {
                if (presraw < xnode[7])  n = 6;
                else
                {
                    n=6;
                    presraw = xnode[7];
                };
            };
        };
        presraw /= 4;
        presraw *= yfac[n];
        presraw /= 256;
        presraw /= 4;
        height_cm = (ysum[n] - presraw);
        //*alti = (ysum[n] - presraw);
    };

};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif





