#ifndef i2c_interface_h
#define i2c_interface_h

// Abstract Class as Interface for sensor-libs
// with basic functions that have to be implemented

class i2cSensor
{
    public:
        virtual void    setEnabled(uint8_t) = 0;            // Enable Sensor or set it to Standby (1 or 0)
        //void            setEnabled(void) {setEnabled(1);};  // just Enable Sensor and don't care for more

        virtual void    reset(void) = 0;                    // trigger a software-reboot of the sensor

        virtual uint8_t initialize(void) = 0;               // set up the sensor for basic operation, IF found at address
        //uint8_t         init(void) {initialize();};         // short form of the function above

        virtual void    getValue(uint8_t buffer[]) = 0;     // get the main values the sensor was made for (barometer gives pressure)

    protected:
        // instance gets control over
        //uint8_t _address;
    private:
        // instance has no control over these
};

#endif
