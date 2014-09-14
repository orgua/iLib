#ifndef i2c_SI7021_h
#define i2c_SI7021_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the SI7021-Sensor

CONSUMPTION: sleep 0.6µA, measure 120-180µA, 3.1mA Heater
MEASUREMENT: 12ms for 12bit RH, 11ms for 14bit Temp --> measurement time is combination of these two
ACCURACY     in Main Range 0.4°C, 3% RH

With every humiditymeasurement there is an automatical temperaturmeasurement (just read it afterwards)

You have to trigger the Measurement yourself --> requestHumidity,

########################################################################  */

class SI7021 : public i2cSensor
{

    /** ######### Register-Map ################################################################# */

#define SI7021_ADDRESS 		        0x40

#define CMD_MEASURE_HUMIDITY_HOLD       0xE5    // Hold:    clock stretching
#define CMD_MEASURE_HUMIDITY_NO_HOLD    0xF5    // No Hold: Not Acknowledging read requests
#define CMD_MEASURE_TEMPERATURE_HOLD    0xE3    // performs an extra measurement
#define CMD_MEASURE_TEMPERATURE_NO_HOLD 0xF3

#define CMD_READ_PREVIOUS_TEMPERATURE   0xE0    // performs NO extra measurement,
#define CMD_RESET                       0xFE
#define CMD_WRITE_REG1                  0xE6
#define CMD_READ_REG1                   0xE7
#define   MASK_RESOLUTION               B10000001
#define     VAL_RESOLUTION_H12_T14      0x00    // takes 10 -12 ms
#define     VAL_RESOLUTION_H11_T11      0x01    // takes 5.8- 7 ms
#define     VAL_RESOLUTION_H10_T13      0x80    // takes 3.7-4.5 ms
#define     VAL_RESOLUTION_H08_T12      0x81    // takes 2.6-3.1 ms
#define   MASK_LOW_VOLTAGE              0x40    // just for reading/checking
#define   MASK_ENABLE_HEATER            0x04    // for dew-point-measurement


    /** ######### function definition ################################################################# */

protected:

    /**< helperfunction */
    uint16_t readValue(uint8_t registerAdd)
    {
        uint16_t _ret;
        Wire.beginTransmission(SI7021_ADDRESS);
        Wire.write(registerAdd);
        Wire.endTransmission();                   // difference to normal read
        Wire.beginTransmission(SI7021_ADDRESS);   // difference to normal read
        Wire.requestFrom(SI7021_ADDRESS, 2);
        _ret      = (Wire.read() << 8 );
        _ret      |= Wire.read();
        Wire.endTransmission();
        return _ret;
    };

public:

    SI7021(void)
    {
        //_address = MPL_ADDRESS;
    };

    /**< resolutions for temp and humidity is combined, influences the duration */
    void setResolution(uint8_t stepH=4)
    {
        switch (stepH)
        {
        case 1:
            i2c.setRegister(SI7021_ADDRESS, CMD_WRITE_REG1, MASK_RESOLUTION, VAL_RESOLUTION_H08_T12);
            break;
        case 2:
            i2c.setRegister(SI7021_ADDRESS, CMD_WRITE_REG1, MASK_RESOLUTION, VAL_RESOLUTION_H10_T13);
            break;
        case 3:
            i2c.setRegister(SI7021_ADDRESS, CMD_WRITE_REG1, MASK_RESOLUTION, VAL_RESOLUTION_H11_T11);
            break;
        default:
            i2c.setRegister(SI7021_ADDRESS, CMD_WRITE_REG1, MASK_RESOLUTION, VAL_RESOLUTION_H12_T14); // takes about 12ms
            break;
        }
    };

    /**< for dew-point measurements, takes 3.1mA */
    void setHeater(uint8_t enable=1)
    {
        if (enable)  i2c.setRegister(SI7021_ADDRESS, CMD_WRITE_REG1, MASK_ENABLE_HEATER, 255);
        else         i2c.setRegister(SI7021_ADDRESS, CMD_WRITE_REG1, MASK_ENABLE_HEATER, 0);
    };

    /**< no function */
    void setEnabled(uint8_t enable=1)
    {

    };

    /**< softwarereset */
    void reset(void)
    {
        i2c.writeCMD(SI7021_ADDRESS, CMD_RESET);
    };

    /**< standardvalues */
    byte initialize()
    {
        if (i2c.probe(SI7021_ADDRESS))
        {
            setHeater(0);
            setResolution(4);
            requestMeasurement();   // Manual Mode: Request and Read
            return 1;
        }
        return 0;
    };

    /**< there is also a temperaturemeasurement taken */
    void requestMeasurement()
    {
        i2c.writeCMD(SI7021_ADDRESS, CMD_MEASURE_HUMIDITY_HOLD);
    };

    /**< inefficient way to get a value */
    float readHumidity(void)
    {
        float     _rh;
        int32_t   _rawHumi;
        _rawHumi  = readValue(CMD_MEASURE_HUMIDITY_HOLD);
        _rh       = (_rawHumi*125.0/65536) - 6;
        return    _rh;
    };

    void getHumidity(float& rh) // pass a reference
    {
        int32_t   _rawHumi;
        _rawHumi  = readValue(CMD_MEASURE_HUMIDITY_HOLD);
        rh        = (_rawHumi*125.0/65536) - 6;
    };

    /**< inefficient way to get a value */
    float readTemperature(void)
    {
        float     _celsius;
        int32_t   _rawTemp;
        _rawTemp  = readValue(CMD_READ_PREVIOUS_TEMPERATURE);
        _celsius  = (_rawTemp*175.72/65536) - 46.85;
        return    _celsius;
    };

    /**< there is an extra measurement triggered */
    void getTemperature(float& celsius) // pass a reference
    {
        int32_t   _rawTemp;
        _rawTemp  = readValue(CMD_READ_PREVIOUS_TEMPERATURE);
        celsius   = (_rawTemp*175.72/65536) - 46.85;
    };

    /**< not very usefull */
    void requestTemperature()
    {
        i2c.writeCMD(SI7021_ADDRESS, CMD_MEASURE_TEMPERATURE_HOLD);
    };

    /**< inefficient way to get a value */
    float readTemperatureReq(void)
    {
        float     _celsius;
        int32_t   _rawTemp;
        _rawTemp  = readValue(CMD_MEASURE_TEMPERATURE_HOLD);
        _celsius  = (_rawTemp*175.72/65536) - 46.85;
        return    _celsius;
    };

    /**< there is an extra measurement triggered */
    void getTemperatureReq(float& celsius) // pass a reference
    {
        int32_t   _rawTemp;
        _rawTemp  = readValue(CMD_MEASURE_TEMPERATURE_HOLD);
        celsius   = (_rawTemp*175.72/65536) - 46.85;
    };


};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PRESET preset = PRESET();

#endif




