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
private:
    /** ######### Register-Map ################################################################# */

const uint8_t  MPL_ADDRESS 	            {0x60};

const uint8_t  REG_STATUS 	            {0x00}; // data ready, previous data unread and overwritten
const uint8_t  REG_OUT_P_MSB 	        {0x01};
const uint8_t  REG_OUT_P_CSB 	        {0x02};
const uint8_t  REG_OUT_P_LSB 	        {0x03};
const uint8_t  REG_OUT_T_MSB 	        {0x04};
const uint8_t  REG_OUT_T_LSB 	        {0x05};
const uint8_t  REG_DR_STATUS 	        {0x06};
const uint8_t  REG_OUT_P_DELTA_MSB 	    {0x07};
const uint8_t  REG_OUT_P_DELTA_CSB 	    {0x08};
const uint8_t  REG_OUT_P_DELTA_LSB 	    {0x09};
const uint8_t  REG_OUT_T_DELTA_MSB      {0x0A};
const uint8_t  REG_OUT_T_DELTA_LSB 	    {0x0B};
const uint8_t  REG_WHO_AM_I   	        {0x0C};
const uint8_t    VAL_WHO_AM_I           {0xC4};

const uint8_t  REG_FIFO_STATUS 	        {0x0D};
const uint8_t  REG_FIFO_DATA 	        {0x0E};
const uint8_t  REG_FIFO_SETUP 	        {0x0F};
const uint8_t  REG_TIME_DLY 	        {0x10};
const uint8_t  REG_SYSMOD 	            {0x11};
const uint8_t    VAL_SYS_ACTIVE         {0x01};
const uint8_t    VAL_SYS_STANDBY        {0x00};

const uint8_t  REG_INT_SOURCE 	        {0x12};
const uint8_t  REG_PT_DATA_CFG 	        {0x13};
const uint8_t    MSK_DATA_READY         {0x04};
const uint8_t    MSK_PRES_READY         {0x02};
const uint8_t    MSK_TEMP_READY         {0x01};

const uint8_t  REG_BAR_IN_MSB 	        {0x14};
const uint8_t  REG_BAR_IN_LSB 	        {0x15};
const uint8_t  REG_P_TGT_MSB 	        {0x16};  // target int16 in meters OR uint16 in 2Pa
const uint8_t  REG_P_TGT_LSB 	        {0x17};
const uint8_t  REG_T_TGT 	            {0x18};
const uint8_t  REG_P_WND_MSB 	        {0x19};
const uint8_t  REG_P_WND_LSB 	        {0x1A};
const uint8_t  REG_T_WND 	            {0x1B};
const uint8_t  REG_P_MIN_MSB 	        {0x1C};
const uint8_t  REG_P_MIN_CSB 	        {0x1D};
const uint8_t  REG_P_MIN_LSB 	        {0x1E};
const uint8_t  REG_T_MIN_MSB 	        {0x1F};
const uint8_t  REG_T_MIN_LSB 	        {0x20};
const uint8_t  REG_P_MAX_MSB 	        {0x21};
const uint8_t  REG_P_MAX_CSB 	        {0x22};
const uint8_t  REG_P_MAX_LSB 	        {0x23};
const uint8_t  REG_T_MAX_MSB 	        {0x24};
const uint8_t  REG_T_MAX_LSB 	        {0x25};

const uint8_t  REG_CTRL1 	            {0x26};  //
const uint8_t    MSK_SBYB               {0x01};
const uint8_t    MSK_OST                {0x02};  // triggers measurement even when standby (onetime)!!!
const uint8_t    MSK_RST                {0x04};
const uint8_t    MSK_OS                 {B00111000};
const uint8_t    MSK_RAW                {B01000000};
const uint8_t    MSK_ALT                {B10000000};

const uint8_t  REG_CTRL2 	            {0x27};  // can only be modified in standby
const uint8_t  REG_CTRL3 	            {0x28};  // can only be modified in standby
const uint8_t  REG_CTRL4 	            {0x29};  // can only be modified in standby
const uint8_t  REG_CTRL5 	            {0x2A};  // can only be modified in standby

const uint8_t  REG_OFF_P 	            {0x2B};  // registers not preserved in standby
const uint8_t  REG_OFF_T 	            {0x2C};  // registers not preserved in standby
const uint8_t  REG_OFF_H 	            {0x2D};  // registers not preserved in standby

    /** ######### function definition ################################################################# */

public:

    MPL3115A2(void)
    {
        //_address = MPL_ADDRESS;
    };

   /**< Enable Altimeter / Barometer MODE */
    void setAltimeter(uint8_t enable = 1)
    {
        if (enable) enable=MSK_ALT;
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
    void setEventFlags(const uint8_t flags)
    {
        i2c.writeByte(MPL_ADDRESS,REG_PT_DATA_CFG, (flags & 0x07)); // Enable all three pressure and temp event flags
    };


    /**< initialize */
    uint8_t initialize()
    {
        if (i2c.probe(MPL_ADDRESS)==0) return 0;

        reset();
        delay(2);
        setEnabled(0);
        setEventFlags((MSK_DATA_READY | MSK_PRES_READY | MSK_TEMP_READY));
        setOversampleRatio(128);
        setAltimeter(1);
        setEnabled(1);
        return 1;
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

    uint8_t checkMeasurement(void)
    {
        uint8_t _val;
        //Wait for PDR bit, indicates we have new pressure data
        i2c.read(MPL_ADDRESS, REG_STATUS, &_val, 1);
        if (_val & B00000010) return 1; // Measurement finished
        else                  return 0;
    };

    /**<  if you started a measurement and want to actively wait for it to finish */
    uint8_t awaitMeasurement(void)
    {
        uint8_t _counter = 0;
        while(checkMeasurement()==0)
        {
            if(++_counter > 250) return 0; //Error out after max of 500ms for a read
            delay(2);
        }
        return 1; // Measurement finished
    };



    /**<  gives the number of meters above sea level */
    void getAltitude(float& meter)
    {
        uint8_t value[3];
        i2c.read(MPL_ADDRESS, REG_OUT_P_MSB, value, 3);  // meter in Q16.4 signed in 3x8bit left
        float tempcsb = (float(value[2]>>4))/16.0;
        meter = (float)( int16_t(value[0] << 8) | int16_t(value[1])) + tempcsb;
    };


    /**<  gives airpressure in Pascal */
    void getPressure(float& pascal)
    {
        // Read pressure registers
        uint8_t value[3];
        i2c.read(MPL_ADDRESS, REG_OUT_P_MSB, value, 3);  // pascal in Q18.2 unsigned in 3x8bit left

        float tempcsb = (float(value[2]>>4))/4.0;
        pascal = (float( uint16_t(value[0] << 8) | value[1]))*4 + tempcsb;
    };

    /**< not so much precision (2bit), mut much faster (26 instructions) */
    void getPressure(uint32_t& pascal)
    {
        // Read pressure registers
        uint8_t value[3];
        i2c.read(MPL_ADDRESS, REG_OUT_P_MSB, value, 3);  // pascal in Q18.2 unsigned in 3x8bit left
        pascal = uint32_t(( (uint32_t(value[0]<<8) | uint32_t(value[1]))<<2) + (value[2]>>6) );
    };

    /**<  gives pressure-values */
    void getMeasurement(float& pascal)
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

};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//MPL3115 mpl3115 = MPL3115();

#endif



