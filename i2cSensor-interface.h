#ifndef i2c-interface_h
#define i2c-interface_h

class i2cSensor
{
    public:
        virtual void    setEnabled(uint8_t) = 0;
        void            setEnabled() {setEnabled(1);}

        virtual void    reset() = 0;

        virtual uint8_t initialize() = 0;
        void            init() {initialize();}

        virtual void    getValue(uint8_t buffer[]) = 0;
    protected:

    private:
        // instance has no control over
}
