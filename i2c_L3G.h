#ifndef i2c_L3G_h
#define i2c_L3G_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the L3G-Sensor

 CONSUMPTION: standby X µA, measure X µA

 Details:


########################################################################  */

class L3G : public i2cSensor
{

    /** ######### Register-Map ################################################################# */

#define L3G4200D_ADDRESS_SA0_LOW    (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH   (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW      (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH     (0xD6 >> 1)

#define L3G_ADDRESS				    L3GD20_ADDRESS_SA0_HIGH

#define L3G_WHO_AM_I                0x0F

#define L3G_CTRL_REG1               0x20

#define		L3G_DATA_RATE_MASK	    B11000000
#define		L3G_DATA_RATE_100HZ	    B00000000
#define		L3G_DATA_RATE_200HZ     B01000000
#define		L3G_DATA_RATE_400HZ     B10000000
#define		L3G_DATA_RATE_800HZ     B11000000

#define		L3G_BANDWIDTH_MASK	    B00110000
#define		L3G_BANDWIDTH_MIN       B00000000
#define		L3G_BANDWIDTH_LOW       B00010000
#define		L3G_BANDWIDTH_MED       B00100000
#define		L3G_BANDWIDTH_HIG       B00110000

#define		L3G_PWRDWN_DIS_MASK     B00001000
#define		L3G_Z_AXIS_EN_MASK	    B00000100
#define		L3G_Y_AXIS_EN_MASK	    B00000010
#define		L3G_X_AXIS_EN_MASK	    B00000001

#define L3G_CTRL_REG2               0x21

#define		L3G_HIGH_PASS_MODE_MASK	B00110000
#define		L3G_HIGH_PASS_MODE_NORR	B00000000
#define		L3G_HIGH_PASS_MODE_REFE	B00010000
#define		L3G_HIGH_PASS_MODE_NORM	B00100000
#define		L3G_HIGH_PASS_MODE_AUTO	B00110000

#define		L3G_HIGH_PASS_FREQ_MASK B00001111
#define		L3G_HIGH_PASS_FREQ_STP9 B00000000
#define		L3G_HIGH_PASS_FREQ_STP8 B00000001
#define		L3G_HIGH_PASS_FREQ_STP7 B00000010
#define		L3G_HIGH_PASS_FREQ_STP6 B00000011
#define		L3G_HIGH_PASS_FREQ_STP5 B00000100
#define		L3G_HIGH_PASS_FREQ_STP4 B00000101
#define		L3G_HIGH_PASS_FREQ_STP3 B00000110
#define		L3G_HIGH_PASS_FREQ_STP2 B00000111
#define		L3G_HIGH_PASS_FREQ_STP1 B00001000
#define		L3G_HIGH_PASS_FREQ_STP0 B00001001

#define L3G_CTRL_REG3               0x22

#define		L3G_INT1_EN_MASK		B10000000
#define		L3G_INT1_BOOTSTAT_MASK	B01000000
#define		L3G_INT1_LOW_ACT_MASK	B00100000
#define		L3G_INT_OPNDRAIN_MASK	B00010000
#define		L3G_DRDY_DRDY_MASK		B00001000
#define		L3G_DRDY_FIFO_WTM_MASK	B00000100
#define		L3G_DRDY_FIFO_ORUN_MASK	B00000010
#define		L3G_DRDY_FIFO_EMTY_MASK	B00000001

#define L3G_CTRL_REG4               0x23

#define		L3G_BLOCK_D_UPDATE_MASK	B10000000
#define		L3G_MSB_ON_LOWADD_MASK	B01000000

#define		L3G_SCALE_MASK			B00110000
#define		L3G_SCALE_0250DPS		B00000000
#define		L3G_SCALE_0500DPS		B00010000
#define		L3G_SCALE_1000DPS		B00100000
#define		L3G_SCALE_2000DPS		B00110000

#define		L3G_SELFTEST_EN_MASK	B00000110
#define		L3G_SELFTEST_DIS		B00000000
#define		L3G_SELFTEST_EN0		B00000010
#define		L3G_SELFTEST_EN1		B00000110

#define		L3G_SPI_TO_3WIRE_MASK	B00000001
#define		L3G_SPI_TO_3WIRE_DIS	B00000000
#define		L3G_SPI_TO_3WIRE_EN		B00000001

#define L3G_CTRL_REG5               0x24

#define		L3G_REBOOT_MEM_MASK		B10000000
#define		L3G_FIFO_EN_MASK		B01000000
#define		L3G_HIGH_PASS_EN_MASK	B00010000
#define		L3G_INTSEL_LPF_EN_MASK	B00001000
#define		L3G_INTSEL_HPF_EN_MASK	B00000100
#define		L3G_DATA_LPF_EN_MASK	B00000010
#define		L3G_DATA_HPF_EN_MASK	B00000001

#define L3G_REFERENCE               0x25
#define L3G_OUT_TEMP                0x26
#define L3G_STATUS_REG              0x27

#define L3G_OUT_X_L                 0x28
#define L3G_OUT_X_H                 0x29
#define L3G_OUT_Y_L                 0x2A
#define L3G_OUT_Y_H                 0x2B
#define L3G_OUT_Z_L                 0x2C
#define L3G_OUT_Z_H                 0x2D

#define L3G_FIFO_CTRL_REG           0x2E
#define L3G_FIFO_SRC_REG            0x2F

#define L3G_INT1_CFG                0x30
#define L3G_INT1_SRC                0x31
#define L3G_INT1_THS_XH             0x32
#define L3G_INT1_THS_XL             0x33
#define L3G_INT1_THS_YH             0x34
#define L3G_INT1_THS_YL             0x35
#define L3G_INT1_THS_ZH             0x36
#define L3G_INT1_THS_ZL             0x37
#define L3G_INT1_DURATION           0x38

    /** ######### function definition ################################################################# */

private:

    float sens_factor;

public:

    L3G(void)
    {
        //_address = MPL_ADDRESS;
    };

    void reset(void)
    {
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_REBOOT_MEM_MASK,	255);
        delay(3);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_REBOOT_MEM_MASK,	0);
    };

    void setEnabled(uint8_t enable=1)
    {
        if (enable) enable = 255;
        else        enable = 0;
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG1, L3G_PWRDWN_DIS_MASK, enable);
    };

    void setDatarate(uint16_t hzFreq = 100)
    {
        uint8_t _value;
        if 		(hzFreq <= 100) _value = L3G_DATA_RATE_100HZ;  //  95Hz
        else if (hzFreq <= 200) _value = L3G_DATA_RATE_200HZ;  // 190Hz
        else if (hzFreq <= 400) _value = L3G_DATA_RATE_400HZ;  // 380Hz
        else  				    _value = L3G_DATA_RATE_800HZ;  // 760Hz
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG1, L3G_DATA_RATE_MASK,	_value);	// 100HZ
    };

    void setSensibility(uint16_t degScaleRange)
    {
        uint8_t _value;
        if 		(degScaleRange <= 250)
        {
            _value      = L3G_SCALE_0250DPS;
            sens_factor = 0.00875;
        }
        else if (degScaleRange <= 500)
        {
            _value      = L3G_SCALE_0500DPS;
            sens_factor = 0.01750;
        }
        //else if (degScaleRange <= 400)	{ value = L3G_SCALE_1000DPS; }
        else
        {
            _value      = L3G_SCALE_2000DPS;
            sens_factor = 1/13.98f; // 0.07000;
        }
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG4, L3G_SCALE_MASK,	_value);
    };

    uint8_t initialize(void)
    {
        return initialize(100, 250);
    }

    uint8_t initialize(const uint16_t hzFreq, const uint16_t degScaleRange)
    {
        if (i2c.probe(L3G_ADDRESS)==0) return 0;

        reset();
        setEnabled(0);
        setDatarate(hzFreq);

        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG1, L3G_BANDWIDTH_MASK, L3G_BANDWIDTH_HIG);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG1, L3G_Z_AXIS_EN_MASK, L3G_Z_AXIS_EN_MASK);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG1, L3G_Y_AXIS_EN_MASK, L3G_Y_AXIS_EN_MASK);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG1, L3G_X_AXIS_EN_MASK, L3G_X_AXIS_EN_MASK);

        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG2, B11000000, 0); // Just for secure operation
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG2, L3G_HIGH_PASS_MODE_MASK, L3G_HIGH_PASS_MODE_NORM);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG2, L3G_HIGH_PASS_FREQ_MASK, L3G_HIGH_PASS_FREQ_STP0);

        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_INT1_EN_MASK,			0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_INT1_BOOTSTAT_MASK,		0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_INT1_LOW_ACT_MASK,		0); // 0=HIGH active
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_INT_OPNDRAIN_MASK,		0); // 0=PushPull
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_DRDY_DRDY_MASK,			255);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_DRDY_FIFO_WTM_MASK,		0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_DRDY_FIFO_ORUN_MASK,	0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG3, L3G_DRDY_FIFO_EMTY_MASK,	0);

        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG4, L3G_BLOCK_D_UPDATE_MASK,	0); // 0=continuos update
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG4, L3G_MSB_ON_LOWADD_MASK,		0);

        setSensibility(degScaleRange);

        //i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG4, L3G_SELFTEST_EN_MASK,	L3G_SELFTEST_DIS); // should not be changed in gd20
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG4, L3G_SPI_TO_3WIRE_MASK,	L3G_SPI_TO_3WIRE_DIS);

        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_FIFO_EN_MASK,		0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_HIGH_PASS_EN_MASK,	0); 	// Combination - not helpfull, or?
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_INTSEL_LPF_EN_MASK,	0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_INTSEL_HPF_EN_MASK,	0);
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_DATA_LPF_EN_MASK,	0);	// Combination
        i2c.setRegister(L3G_ADDRESS, L3G_CTRL_REG5, L3G_DATA_HPF_EN_MASK,	0); 	// Combination

        setEnabled(1);

        return 1;
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

    /**< Reads the 3 gyro channels RAW!!!

        Sensitivity for the L3GD20:
            250dps --> 8.75 mdps/digit
            500dps --> 17.5 mdps/digit
            2000dps --> 70.0 mdps/digit
     */
    void getMeasurement(int16_t xyz_raw[])
    {
        byte _byte[6];

        i2c.read(L3G_ADDRESS, (L3G_OUT_X_L | (1 << 7)), _byte, 6);

        // combine high and low bytes
        xyz_raw[0]  = (int16_t)(_byte[1] << 8 | _byte[0]); // X
        xyz_raw[1]  = (int16_t)(_byte[3] << 8 | _byte[2]); // Y
        xyz_raw[2]  = (int16_t)(_byte[5] << 8 | _byte[4]); // Z
    };

    /**< with sensitivity in dps */
    void getMeasurement(float xyz_dps[])
    {
        byte _byte[6];

        i2c.read(L3G_ADDRESS, (L3G_OUT_X_L | (1 << 7)), _byte, 6);

        // combine high and low bytes
        xyz_dps[0]  = ((float)(_byte[1] << 8 | _byte[0]))*sens_factor; // X
        xyz_dps[1]  = ((float)(_byte[3] << 8 | _byte[2]))*sens_factor; // Y
        xyz_dps[2]  = ((float)(_byte[5] << 8 | _byte[4]))*sens_factor; // Z
    };

};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif




