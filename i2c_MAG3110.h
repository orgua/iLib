#ifndef i2c_MAG3110_h
#define i2c_MAG3110_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the MAG3310-Sensor

 CONSUMPTION: standby 2 µA, measure 9-900 µA

 Details: up to 80Hz, ±1000 µT, Sensitivity of 0.10 µT


########################################################################  */

class MAG3110 : public i2cSensor, public manualSensor
{

    /** ######### Register-Map ################################################################# */

#define MAG3110_ADDRESS         0x0E

#define MAG3110_REG_DR_STATUS	0x00
#define MAG3110_REG_OUT_X_MSB	0x01
#define MAG3110_REG_OUT_X_LSB	0x02
#define MAG3110_REG_OUT_Y_MSB	0x03
#define MAG3110_REG_OUT_Y_LSB	0x04
#define MAG3110_REG_OUT_Z_MSB	0x05
#define MAG3110_REG_OUT_Z_LSB	0x06

#define MAG3110_REG_WHO_AM_I	0x07
#define     MAG_VAL_WHO_AM_I    0xC4
#define MAG3110_REG_SYSMOD		0x08

#define MAG3110_REG_OFF_X_MSB	0x09
#define MAG3110_REG_OFF_X_LSB	0x0A
#define MAG3110_REG_OFF_Y_MSB	0x0B
#define MAG3110_REG_OFF_Y_LSB	0x0C
#define MAG3110_REG_OFF_Z_MSB	0x0D
#define MAG3110_REG_OFF_Z_LSB	0x0E

#define MAG3110_REG_DIE_TEMP	0x0F
#define MAG3110_REG_CTRL_REG1	0x10
#define     MASK_ACTIVE		    0x01        // standby --> active
#define     MASK_TM		        0x02        // Trigger Measurement
#define     MASK_FR             0x04        // Fast read: skip low-registers
#define     MASK_OSR            B00011000   // Oversampling
#define         VAL_OSR_16	    B00000000
#define         VAL_OSR_32	    B00001000
#define         VAL_OSR_64		B00010000
#define         VAL_OSR_128		B00011000
#define     MASK_DR             B11100000   // datarate
#define         VAL_DR_0	    B00000000
#define         VAL_DR_1	    B00100000
#define         VAL_DR_2	    B01000000
#define         VAL_DR_3	    B01100000
#define         VAL_DR_4	    B10000000
#define         VAL_DR_5	    B10100000
#define         VAL_DR_6	    B11000000
#define         VAL_DR_7	    B11100000

#define MAG3110_REG_CTRL_REG2	0x11
#define     MASK_AUTO_MRST	    B10000000   // Magnetic Sensor reset
#define     MASK_RAW			B00100000   // read uncorrected RAW values
#define     MASK_RESET		    B00010000   // Trigger a Reset

#define SYSMOD_STANDBY		    0x00
#define SYSMOD_ACTIVE_RAW	    0x01
#define SYSMOD_ACTIVE		    0x02

    /** ######### function definition ################################################################# */

public:
    /**< TODO: do i need a constructor? */
    MAG3110(void)
    {
        //_address = MPL_ADDRESS;
    };

    /**<  Trigger a software-reset*/
    void reset(void)
    {
        i2c.setRegister(MAG3110_ADDRESS,MAG3110_REG_CTRL_REG2,MASK_RESET, 255);
    };

    /**<  recommended: the resets occur automatically before each data acquisition*/
    void setSensorAutoReset(uint8_t enable=1)
    {
        if (enable) enable = MASK_AUTO_MRST;
        else        enable = 0;
        i2c.setRegister(MAG3110_ADDRESS,MAG3110_REG_CTRL_REG2,MASK_AUTO_MRST, enable);
    };

    /**< data values are not corrected by the user offset register values. */
    void setRawMode(uint8_t enable=1)
    {
        if (enable) enable = MASK_RAW;
        else        enable = 0;
        i2c.setRegister(MAG3110_ADDRESS,MAG3110_REG_CTRL_REG2,MASK_RAW, enable);
    };

    /**< switch between continious and trigger/manual mode */
    void setEnabled(uint8_t enable=1)
    {
        if (enable) enable = MASK_ACTIVE;
        else        enable = 0;
        i2c.setRegister(MAG3110_ADDRESS,MAG3110_REG_CTRL_REG1,MASK_ACTIVE, enable);
    };

    void setDataRate(uint8_t hz=10)
    {
        // use modes with 1280 Hz ADC Rate
        if 		(hz > 40)   hz =  0;  // 80 Hz with 16x OS
        else if (hz > 20) 	hz =  1;  // 40 Hz with 32x OS
        else if (hz > 10) 	hz =  2;  // 20 Hz with 64x OS
        else if (hz >  5)   hz =  3;  // 10 Hz with 128x OS
        else if (hz >= 3)   hz =  7;  //  5 Hz with 128x OS, 640 Hz ADC
        else                hz = 11;  // 2.5Hz with 128x OS, 320 Hz ADC
        i2c.setRegister(MAG3110_ADDRESS, MAG3110_REG_CTRL_REG1, (MASK_DR | MASK_OSR), hz<<3); // set combination of data rate and oversampling
    }

    uint8_t initialize(void)
    {
        return initialize(100);
    };

    uint8_t initialize(uint8_t hzFreq)
    {
        if (i2c.probe(MAG3110_ADDRESS)==0) return 0;

        setEnabled(0);

        setSensorAutoReset(1);
        setRawMode(1);
        setDataRate(10);

        setEnabled(1);

        return 1;
    };

    /**< only used when in manual/standby mode */
    void triggerMeasurement(void)
    {
        i2c.setRegister(MAG3110_ADDRESS, MAG3110_REG_CTRL_REG1, MASK_TM, MASK_TM);
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

    /**< get RAW values */
    void getMeasurement(int16_t xyz_raw[])
    {
        byte buf[6];
        i2c.read(MAG3110_ADDRESS, MAG3110_REG_OUT_X_MSB, buf, 6);
        xyz_raw[0] = (buf[0]<<8) | buf[1];
        xyz_raw[1] = (buf[2]<<8) | buf[3];
        xyz_raw[2] = (buf[4]<<8) | buf[5];
    };

    /**< values scaled to uT */
    void getMeasurement(float xyz_uT[])
    {
        byte buf[6];
        i2c.read(MAG3110_ADDRESS, MAG3110_REG_OUT_X_MSB, buf, 6);
        xyz_uT[0] = int16_t((buf[0]<<8) | buf[1])*0.2471f;
        xyz_uT[1] = int16_t((buf[2]<<8) | buf[3])*0.2479f;
        xyz_uT[2] = int16_t((buf[4]<<8) | buf[5])*0.2480f;
    };


};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif




