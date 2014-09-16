#ifndef i2c_mpl3115a2_h
#define i2c_mpl3115a2_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the MPL3115A2-Sensor

 CONSUMPTION: standby 2µA, measure 260-2000µA

 ONE-TIME-MEASURE: disable sensor, [start measurement, wait, read ] ...
 AUTO-Measure: enable sensor, start measurement, [read, read ] ...


########################################################################  */

class MPL3115A2 : public i2cSensor, public manualSensor
{

    /** ######### Register-Map ################################################################# */

#define MPL_ADDRESS 	        0x60

#define REG_STATUS 	            0x00 // data ready, previous data unread and overwritten
#define REG_OUT_P_MSB 	        0x01
#define REG_OUT_P_CSB 	        0x02
#define REG_OUT_P_LSB 	        0x03
#define REG_OUT_T_MSB 	        0x04
#define REG_OUT_T_LSB 	        0x05
#define REG_DR_STATUS 	        0x06
#define REG_OUT_P_DELTA_MSB 	0x07
#define REG_OUT_P_DELTA_CSB 	0x08
#define REG_OUT_P_DELTA_LSB 	0x09
#define REG_OUT_T_DELTA_MSB     0x0A
#define REG_OUT_T_DELTA_LSB 	0x0B
#define REG_WHO_AM_I   	        0x0C
#define   VAL_WHO_AM_I          0xC4

#define REG_FIFO_STATUS 	    0x0D
#define REG_FIFO_DATA 	        0x0E
#define REG_FIFO_SETUP 	        0x0F
#define REG_TIME_DLY 	        0x10
#define REG_SYSMOD 	            0x11
#define   VAL_SYS_ACTIVE        0x01
#define   VAL_SYS_STANDBY       0x00

#define REG_INT_SOURCE 	        0x12
#define REG_PT_DATA_CFG 	    0x13
#define   MSK_DATA_READY        0x04
#define   MSK_PRES_READY        0x02
#define   MSK_TEMP_READY        0x01

#define REG_BAR_IN_MSB 	        0x14
#define REG_BAR_IN_LSB 	        0x15
#define REG_P_TGT_MSB 	        0x16   // target int16 in meters OR uint16 in 2Pa
#define REG_P_TGT_LSB 	        0x17
#define REG_T_TGT 	            0x18
#define REG_P_WND_MSB 	        0x19
#define REG_P_WND_LSB 	        0x1A
#define REG_T_WND 	            0x1B
#define REG_P_MIN_MSB 	        0x1C
#define REG_P_MIN_CSB 	        0x1D
#define REG_P_MIN_LSB 	        0x1E
#define REG_T_MIN_MSB 	        0x1F
#define REG_T_MIN_LSB 	        0x20
#define REG_P_MAX_MSB 	        0x21
#define REG_P_MAX_CSB 	        0x22
#define REG_P_MAX_LSB 	        0x23
#define REG_T_MAX_MSB 	        0x24
#define REG_T_MAX_LSB 	        0x25

#define REG_CTRL1 	            0x26  //
#define   MSK_SBYB              0x01
#define   MSK_OST               0x02  // triggers measurement even when standby (onetime)!!!
#define   MSK_RST               0x04
#define   MSK_OS                (B00111000)
#define   MSK_RAW               (B01000000)
#define   MSK_ALT               (B10000000)

#define REG_CTRL2 	            0x27  // can only be modified in standby
#define REG_CTRL3 	            0x28  // can only be modified in standby
#define REG_CTRL4 	            0x29  // can only be modified in standby
#define REG_CTRL5 	            0x2A  // can only be modified in standby

#define REG_OFF_P 	            0x2B  // registers not preserved in standby
#define REG_OFF_T 	            0x2C  // registers not preserved in standby
#define REG_OFF_H 	            0x2D  // registers not preserved in standby

    /** ######### function definition ################################################################# */

public:

    MPL3115A2(void)
    {
        //_address = MPL_ADDRESS;
    };

    /**< Clears and sets the OST bit --> immediately take another reading */
    /**< Needed to sample faster than 1Hz */
    void triggerMeasurement(void)
    {
        byte setting = i2c.readByte(MPL_ADDRESS,REG_CTRL1); //Read current settings
        if (setting&2)
        {
            setting &= ~(1<<1); //Clear OST bit
            i2c.writeByte(MPL_ADDRESS,REG_CTRL1, setting);
            setting = i2c.readByte(MPL_ADDRESS,REG_CTRL1); //Read current settings to be safe
        }
        setting |= (1<<1); //Set OST bit
        i2c.writeByte(MPL_ADDRESS,REG_CTRL1, setting);
    };

    /**<  if you started a measurement and want to actively wait for it to finish */
    uint8_t waitMeasurement(void)
    {
        uint16_t counter = 0;
        //Wait for PDR bit, indicates we have new pressure data
        while( (i2c.readByte(MPL_ADDRESS,REG_STATUS) & (1<<1)) == 0)
        {
            if(++counter > 600) return 0; //Error out after max of 512ms for a read
            delay(1);
        }
        return 1;
    };

    /**<  gives the number of meters above sea level */
    void getAltitude(float& meter)
    {
        // Read pressure registers
        uint8_t value[3];
        i2c.read(MPL_ADDRESS, REG_OUT_P_MSB, value, 3);  // meter in Q16.4 signed in 3x8bit left
        float tempcsb = (value[2]>>4)/16.0;
        meter = (float)( (value[0] << 8) | value[1]) + tempcsb;
    };


    /**<  gives airpressure in Pascal */
    void getPressure(float& pascal)
    {
        // Read pressure registers
        uint8_t value[3];
        i2c.read(MPL_ADDRESS, REG_OUT_P_MSB, value, 3);  // pascal in Q18.2 unsigned in 3x8bit left

        float tempcsb = (value[2]>>4)/4.0;
        pascal = (float)( (value[0] << 8) | value[1]) + tempcsb;
    };

    /**<  gives pressure-values */
    void getValue(float& pascal)
    {
        getPressure(pascal);
    };

    /**<  gives temperature in degree celsius */
    void getTemperature(float& celsius)
    {
        // Read temperature registers
        byte value[2];
        i2c.read(MPL_ADDRESS, REG_OUT_T_MSB, value, 2); // °C in Q8.4 signed in 2x

        uint16_t foo;
        bool negSign = false;
        if(value[0] > 0x7F) //Check for 2s compliment
        {
            foo = ~((value[0] << 8) + value[1]) + 1;
            negSign = true;
        }
        else
        {
            foo = ((value[0] << 8) + value[1]);
        }
        celsius = ((float)(foo))/256.0;
        if (negSign) celsius = 0 - celsius;

    };


    /**< Enable Altimeter / Barometer MODE */
    void setAltimeter(uint8_t enable = 1)
    {
        if (enable) enable=(1<<7);
        i2c.setRegister(MPL_ADDRESS,REG_CTRL1, MSK_ALT, enable);
    };

    /**< Enable / Disable the Sensor */
    void    setEnabled(uint8_t enable = 1)
    {
        if (enable) enable=1;
        i2c.setRegister(MPL_ADDRESS,REG_CTRL1, MSK_SBYB, enable);
    };

    /**< read Enable / Disable - Status */
    uint8_t getEnabled()
    {
        return (1 & i2c.readByte(MPL_ADDRESS,REG_SYSMOD));
    };

    /**< do a software reset */
    void    reset()
    {
        i2c.writeByte(MPL_ADDRESS,REG_CTRL1, MSK_RST);
    };

    /**<  */
    uint8_t setOversampleRatio(uint8_t sampleRatio = 128)
    {
        uint8_t ratio;
        if (sampleRatio > 127)
        {
            sampleRatio = 7;    // takes 512ms
            ratio = 128;
        }
        else if (sampleRatio > 63)
        {
            sampleRatio = 6;    // takes 258ms
            ratio = 64;
        }
        else if (sampleRatio > 31)
        {
            sampleRatio = 5;    // takes 130ms
            ratio = 32;
        }
        else if (sampleRatio > 15)
        {
            sampleRatio = 4;    // takes  66ms
            ratio = 16;
        }
        else if (sampleRatio > 7)
        {
            sampleRatio = 3;    // takes  34ms
            ratio = 8;
        }
        else if (sampleRatio > 3)
        {
            sampleRatio = 2;    // takes  18ms
            ratio = 4;
        }
        else if (sampleRatio > 1)
        {
            sampleRatio = 1;    // takes  10ms
            ratio = 2;
        }
        else
        {
            sampleRatio = 0;    // takes   6ms
            ratio = 1;
        }
        sampleRatio <<= 3; //Align it for the CTRL_REG1 register
        i2c.setRegister(MPL_ADDRESS,REG_CTRL1, MSK_OS, sampleRatio); //Read current settings
        return ratio;
    };

    /**< Enables the measurement event flags */
    /**< This is recommended in datasheet during setup. */
    void setEventFlags(uint8_t flags = (MSK_DATA_READY | MSK_PRES_READY | MSK_TEMP_READY))
    {
        flags = flags & 0x07;
        i2c.writeByte(MPL_ADDRESS,REG_PT_DATA_CFG, flags); // Enable all three pressure and temp event flags
    };

    /**< check for new data, return 1 when Measurement is ready */
    uint8_t checkMeasurement(void)
    {
        /**< TODO: Implement */
        return 1; // Measurement finished
    };

    /**<  wait for new data*/
    uint8_t awaitMeasurement(void)
    {
        /**< TODO: Implement */
        return 1; // Measurement finished
    };


    /**< initialize */
    uint8_t initialize()
    {
        if (i2c.probe(MPL_ADDRESS)==0) return 0;

        reset();
        delay(2);
        setEnabled(0);
        setEventFlags();
        setOversampleRatio(128);
        setAltimeter(1);
        setEnabled(1);
        return 1;
    };



};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//MPL3115 mpl3115 = MPL3115();

#endif



