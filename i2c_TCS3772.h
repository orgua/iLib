#ifndef i2c_TCS3772_h
#define i2c_TCS3772_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the TCS3772-Sensor

CONSUMPTION: sleep 4.5µA, wait 65µA, measure 230-330µA
possible gains: 4 (3.8 .. 4.2), 16 (15 .. 16.8), 60 (58 .. 63)


########################################################################  */

class TCS3772 : public i2cSensor
{

    /** ######### Register-Map ################################################################# */

#define TCS_ADDRESS 				0x29

// Keept Out: Proximity Feature, Interrupts,

#define CMD_REPEAT                  (B10000000)
#define CMD_INCREMENT               (B10100000)
#define CMD_SPECIAL                 (B11100000)

#define REG_ENABLE				    0x00
#define   MASK_PON 			  	    (bit(0))   // POWER ON
#define     VAL_PWR_ON				0x01
#define     VAL_PWR_OFF				0x00
#define   MASK_AEN                  (bit(1))   // RGBC-Sensor Enable
#define   MASK_WEN                  (bit(3))   // WAIT Enable

#define REG_ATIME				    0x01       // Integration time in 2.4ms Steps
#define   VAL_MIN                   0xFF       // 2.4ms
#define   VAL_MAX                   0x00       // 614ms

#define   ATIME_TO_MSEC(atime)		(2.4*(256-ATIME))
#define   MSEC_TO_ATIME(msec) 		(256 - msec/2.4)
#define   FACT_TO_ATIME(factor)  	(256 - factor)

#define REG_WTIME                   0x03       // WLONG=0 2.4ms Steps, WLONG=1 28.8ms Steps
#define   MSEC_TO_ATIMELONG(msec)   (256 - msec/28.8)

//// INTERRUPT THRESHOLD, ....

#define REG_CONFIG                  0x0D
#define   MASK_WLONG               (bit(1))    // Factor 12x for WTIME
#define REG_CONTROL				    0x0F
#define   MASK_AGAIN				0x03
#define     VAL_AGAIN_01 			0x00
#define     VAL_AGAIN_04 			0x01
#define     VAL_AGAIN_16 			0x02
#define     VAL_AGAIN_60 			0x03

#define REG_ID   				    0x12
#define   VAL_DEVICE_ID_A 			0x48         // TCS37721 & TCS37725
#define   VAL_DEVICE_ID_B 			0x49	     // TCS37723 & TCS37727

#define REG_STATUS   				0x13
#define   MASK_AVALID               (bit(0))     // cylce completed since AEN=1

#define REG_CLEAR_DATAL 			0x14
#define REG_CLEAR_DATAH 			0x15
#define REG_RED_DATAL 				0x16
#define REG_RED_DATAH 				0x17
#define REG_GREEN_DATAL		  	    0x18
#define REG_GREEN_DATAH			    0x19
#define REG_BLUE_DATAL 				0x1A
#define REG_BLUE_DATAH 				0x1B


    /** ######### function definition ################################################################# */

public:

    TCS3772(void)
    {
        //_address = MPL_ADDRESS;
    };

    void setEnabled(uint8_t enable = 1)
    {
        if (enable) enable = VAL_PWR_ON | MASK_AEN;
        else        enable = 0;
        i2c.setRegister(TCS_ADDRESS, REG_ENABLE | CMD_INCREMENT, MASK_AEN | MASK_PON, enable);
    };

    void reset() {}

    void setWaitTime(uint16_t wait = 0)
    {
        /**< TODO: rewrite with local Vars and only 3 writeCalls at the end */
        if (wait > 7372)
        {
            i2c.setRegister(TCS_ADDRESS, REG_ENABLE | CMD_INCREMENT, MASK_WEN,   255);
            i2c.setRegister(TCS_ADDRESS, REG_CONFIG | CMD_INCREMENT, MASK_WLONG, 255);
            i2c.writeByte(  TCS_ADDRESS, REG_WTIME  | CMD_INCREMENT, VAL_MAX);
        }
        else if (wait > 614)
        {
            i2c.setRegister(TCS_ADDRESS, REG_ENABLE | CMD_INCREMENT, MASK_WEN,   255);
            i2c.setRegister(TCS_ADDRESS, REG_CONFIG | CMD_INCREMENT, MASK_WLONG, 255);
            i2c.writeByte(  TCS_ADDRESS, REG_WTIME  | CMD_INCREMENT, MSEC_TO_ATIMELONG(wait));
        }
        else if (wait < 4)
        {
            i2c.setRegister(TCS_ADDRESS, REG_ENABLE | CMD_INCREMENT, MASK_WEN,   0);
            i2c.setRegister(TCS_ADDRESS, REG_CONFIG | CMD_INCREMENT, MASK_WLONG, 0);
            i2c.writeByte(  TCS_ADDRESS, REG_WTIME  | CMD_INCREMENT, VAL_MIN);
        }
        else
        {
            i2c.setRegister(TCS_ADDRESS, REG_ENABLE | CMD_INCREMENT, MASK_WEN,   255);
            i2c.setRegister(TCS_ADDRESS, REG_CONFIG | CMD_INCREMENT, MASK_WLONG, 0);
            i2c.writeByte(  TCS_ADDRESS, REG_WTIME  | CMD_INCREMENT, MSEC_TO_ATIME(wait));
        }
    };

    void setATime(const uint16_t integrationtime)
    {
        uint8_t _value;
        if (integrationtime > 614)      _value = VAL_MAX;
        else if (integrationtime < 4)   _value = VAL_MIN;
        else                            _value = MSEC_TO_ATIME(integrationtime);
        i2c.writeByte( TCS_ADDRESS, REG_ATIME  | CMD_INCREMENT, _value);
    };

    uint8_t setAGain(const uint8_t gain)
    {
        /**< TODO: rewrite with local Vars and only 1 writeCall at the end */
        if      (gain <  4)
        {
            i2c.setRegister(TCS_ADDRESS, REG_CONTROL | CMD_INCREMENT, MASK_AGAIN, VAL_AGAIN_01);
            return 1;
        }
        else if (gain < 16)
        {
            i2c.setRegister(TCS_ADDRESS, REG_CONTROL | CMD_INCREMENT, MASK_AGAIN, VAL_AGAIN_04);
            return 4;
        }
        else if (gain < 60)
        {
            i2c.setRegister(TCS_ADDRESS, REG_CONTROL | CMD_INCREMENT, MASK_AGAIN, VAL_AGAIN_16);
            return 16;
        }
        else
        {
            i2c.setRegister(TCS_ADDRESS, REG_CONTROL | CMD_INCREMENT, MASK_AGAIN, VAL_AGAIN_60);
            return 60;
        }

    };

    /**< gives back the total gain when values are in good range, otherwise return 0!  */
    uint8_t autoGain(uint16_t val_clear)
    {
        /**< TODO: something is wrong here! switching integrationtime shows no faster measurement?!? */
        static uint8_t val_gain, val_time, gain;
        static uint16_t val_last;

        // val_clear between 0 .. 65k
#define MARGIN_LOW   5000
#define MARGIN_HIGH  (0xFFFF - MARGIN_LOW)

        // val_gain: 0=G1,  1=G4,   2=G16, 3=G60
        // val_time: 0=i64, 1=i128, 2=i256
        if ((val_clear != val_last) || (val_clear == 0xFFFF))
        {
            val_last = val_clear;

            if      (val_clear < MARGIN_LOW)
            {

                if (val_gain<3)
                {
                    val_gain++;
                    gain = 1<<(2*val_gain);
                    gain = setAGain(gain);
                    return 0;
                }
                else if ( val_time < 2)
                {
                    val_time++;
                    uint16_t time;
                    time = 1<<(6+val_time);
                    setATime(2.4*time);
                    return 0;
                }

            }
            else if (val_clear > MARGIN_HIGH)
            {
                if (val_time>0)
                {
                    val_time--;
                    uint16_t time;
                    time = 1<<(6+val_time);
                    setATime(2.4*time);
                    return 0;
                }
                else if (val_gain > 0)
                {
                    val_gain--;
                    gain = 1<<(2*val_gain);
                    gain = setAGain(gain);
                    return 0;
                };
            }
            return (gain*(1<<val_time));
        }

        return 0;

        /* OLD CODE - doesnt wait for good measurement-range
          if      (val_clear < MARGIN_LOW)  { if (val_gain<3) { val_gain++; } else if ( val_time < 2) { val_time++; }}
          else if (val_clear > MARGIN_HIGH) { if (val_time>0) { val_time--; } else if ( val_gain > 0) { val_gain--; }}
        }
        uint16_t time;
        time = 1<<(6+val_time);
        tcs_setATime(2.4*time);
        gain = 1<<(2*val_gain);
        gain = tcs_setAGain(gain);
        return (gain*(1<<val_time));
        */

    };


    uint8_t initialize()
    {
        if (i2c.probe(TCS_ADDRESS)==0) return 0;

        byte _sensor_id;
        _sensor_id = i2c.readByte(TCS_ADDRESS, REG_ID | CMD_INCREMENT);

        setEnabled(1);
        setATime(2.4*64);
        setAGain(1);
        setWaitTime(0);

        if      (_sensor_id == VAL_DEVICE_ID_A) return 1;
        else if (_sensor_id == VAL_DEVICE_ID_B) return 1;
        else                                    return 0;
    }

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

    void getMeasurement(uint16_t value_crgb[])
    {
        uint8_t _content[8];
        i2c.read(TCS_ADDRESS, REG_CLEAR_DATAL | CMD_INCREMENT, _content, 8);
        value_crgb[0] = (_content[1]<<8) + _content[0];
        value_crgb[1] = (_content[3]<<8) + _content[2];
        value_crgb[2] = (_content[5]<<8) + _content[4];
        value_crgb[3] = (_content[7]<<8) + _content[6];
    }

};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif




