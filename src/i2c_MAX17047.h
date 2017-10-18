#ifndef i2c_MAX17047_h
#define i2c_MAX17047_h

#include "i2c.h"

///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////  MAX17047 ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

class MAX17047
{

    /** ######### Register-Map ################################################################# */
public:

    static constexpr uint8_t I2C_ADDRESS                    	{0x36};

private:

    float   MAX17047_SENSE                     {0.01}; // Application-specific sense resistor. Refer to MAX17047 documentation.

    static constexpr uint8_t MAX17047_STATUS 					{0x00};  // Full, Empty, Normal
    static constexpr uint8_t MAX17047_V_ALRT_THRESHOLD 	        {0x01};
    static constexpr uint8_t MAX17047_T_ALRT_THRESHOLD 	        {0x02};
    static constexpr uint8_t MAX17047_SOC_ALRT_THRESHOLD	    {0x03};
    static constexpr uint8_t MAX17047_AT_RATE					{0x04};
    static constexpr uint8_t MAX17047_REM_CAP_REP			    {0x05};  // Capacity in uVh / filtered -AV
    static constexpr uint8_t MAX17047_SOC_REP				    {0x06};  // State of Charge in % (Highbyte) Filtered -AV
    static constexpr uint8_t MAX17047_AGE						{0x07};  // FULL_CAP divided by DESIGN_CAP * 100%. Also known as state of health.
    static constexpr uint8_t MAX17047_TEMPERATURE			    {0x08};
    static constexpr uint8_t MAX17047_V_CELL					{0x09};
    static constexpr uint8_t MAX17047_CURRENT				    {0x0A};
    static constexpr uint8_t MAX17047_AVERAGE_CURRENT		    {0x0B};  // Average over a user-defined period
    static constexpr uint8_t MAX17047_SOC_MIX			        {0x0D};  // State of Charge in % (Highbyte)
    static constexpr uint8_t MAX17047_SOC_AV					{0x0E};    // State of Charge in % (Highbyte) considering all information
    static constexpr uint8_t MAX17047_REM_CAP_MIX			    {0x0F};    // Capacity in uVh (div Senseresistor for mAh)
    static constexpr uint8_t MAX17047_FULL_CAP				    {0x10};    // best case full capacity in uVh
    static constexpr uint8_t MAX17047_TTE						{0x11};    // Time to Empty 5.625s/LSB
    static constexpr uint8_t MAX17047_Q_RESIDUAL_00			    {0x12};
    static constexpr uint8_t MAX17047_FULL_SOC_THR			    {0x13};
    static constexpr uint8_t MAX17047_AVERAGE_TEMP			    {0x16};
    static constexpr uint8_t MAX17047_CYCLES					{0x17};    // accumulate total percent in Change in %
    static constexpr uint8_t MAX17047_DESIGN_CAP				{0x18};   // Application-specific input
    static constexpr uint8_t MAX17047_AVERAGE_V_CELL		    {0x19};
    static constexpr uint8_t MAX17047_MAX_MIN_TEMP			    {0x1A};
    static constexpr uint8_t MAX17047_MAX_MIN_VOLTAGE		    {0x1B};
    static constexpr uint8_t MAX17047_MAX_MIN_CURRENT		    {0x1C};
    static constexpr uint8_t MAX17047_CONFIG					{0x1D};
    static constexpr uint8_t MAX17047_I_CHG_TERM				{0x1E};
    static constexpr uint8_t MAX17047_REM_CAP_AV				{0x1F};
    static constexpr uint8_t MAX17047_VERSION					{0x21};
    static constexpr uint8_t MAX17047_Q_RESIDUAL_10			    {0x22};
    static constexpr uint8_t MAX17047_FULL_CAP_NOM			    {0x23};
    static constexpr uint8_t MAX17047_TEMP_NOM				    {0x24};
    static constexpr uint8_t MAX17047_TEMP_LIM				    {0x25};
    static constexpr uint8_t MAX17047_AIN						{0x27};
    static constexpr uint8_t MAX17047_LEARN_CFG				    {0x28};
    static constexpr uint8_t MAX17047_FILTER_CFG				{0x29};
    static constexpr uint8_t MAX17047_RELAX_CFG				    {0x2A};
    static constexpr uint8_t MAX17047_MISC_CFG				    {0x2B};
    static constexpr uint8_t MAX17047_T_GAIN					{0x2C};
    static constexpr uint8_t MAX17047_T_OFF					    {0x2D};
    static constexpr uint8_t MAX17047_C_GAIN					{0x2E};
    static constexpr uint8_t MAX17047_C_OFF					    {0x2F};
    static constexpr uint8_t MAX17047_Q_RESIDUAL_20			    {0x32};
    static constexpr uint8_t MAX17047_I_AVG_EMPTY			    {0x36};
    static constexpr uint8_t MAX17047_F_CTC					    {0x37};
    static constexpr uint8_t MAX17047_RCOMP_0				    {0x38};
    static constexpr uint8_t MAX17047_TEMP_CO				    {0x39};
    static constexpr uint8_t MAX17047_V_EMPTY					{0x3A};    // Empty Voltage
    static constexpr uint8_t MAX17047_F_STAT					{0x3D};
    static constexpr uint8_t MAX17047_TIMER					    {0x3E};
    static constexpr uint8_t MAX17047_SHDN_TIMER				{0x3F};
    static constexpr uint8_t MAX17047_Q_RESIDUAL_30			    {0x42};
    static constexpr uint8_t MAX17047_D_QACC					{0x45};
    static constexpr uint8_t MAX17047_D_PACC					{0x46};
    static constexpr uint8_t MAX17047_QH						{0x4D};
    static constexpr uint8_t MAX17047_V_FOCV					{0xFB};
    static constexpr uint8_t MAX17047_SOC_VF					{0xFF};    // State of Charge according to voltage fuel gauge

    MAX17047(const MAX17047&);            // declaration only for copy constructor
    MAX17047& operator=(const MAX17047&);  // declaration only for copy assignment --> make it uncopyable

public:

    MAX17047(void)
    {

    };

    // datasheet says to wait ~600ms before working with the IC if POR=1
    bool getPORStatus(void) const
    {
        // Check POR-Status
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_STATUS, value, 2);
        if ((value[0] & 2))     return true;
        else                    return false;
    }

    void clearPORStatus(void) const
    {
        // clear POR-bit
        uint8_t value[2];
        value[0] &= 0b11111101; // POR
        i2c.write(I2C_ADDRESS, MAX17047_STATUS, value, 2);
    }

    // basic initialization
    void setBasicConfig() const
    {
        uint8_t  uvalue[2];

        // Programm Alert to SOC_AV
        i2c.read(I2C_ADDRESS, MAX17047_MISC_CFG, uvalue, 2);
        uvalue[0] &= 0b11111100;
        uvalue[0] |= 0b00000001;
        i2c.write(I2C_ADDRESS, MAX17047_MISC_CFG, uvalue, 2);

        // MAX17047_CONFIG
        i2c.read(I2C_ADDRESS, MAX17047_CONFIG, uvalue, 2);
        // Enable Alarms
        uvalue[0] |= 0b00000100; // Aen
        uvalue[1] &= 0b11110111; // ALRTp

        // disable Thermistor
        uvalue[0] &= 0b11101111; // ETHRM
        i2c.write(I2C_ADDRESS, MAX17047_CONFIG, uvalue, 2);
    };

    // application specific (only set on POR)
    void setCapacityDesign(const uint16_t capacity_mA) const
    {
        uint16_t uvalue = uint16_t(float(capacity_mA) * (MAX17047_SENSE * 200.0f)); // mAh * R / uVh
        uint8_t  value[2];
        value[0] = uvalue & 0xFF;
        value[1] = (uvalue>>8) & 0xFF;
        i2c.write(I2C_ADDRESS, MAX17047_DESIGN_CAP, value, 2);
    };

    // application specific (only set on POR)
    void setFullThreshold(const uint8_t percent_soc) const
    {
        uint8_t  value[2];
        value[0] = 0;
        value[1] = (percent_soc) & 0xFF;
        i2c.write(I2C_ADDRESS, MAX17047_FULL_SOC_THR, value, 2);
    };

    // application specific (only set on POR)
    void setTerminationChargeCurrent(const uint16_t current_mA) const
    {
        uint8_t  value[2];
        uint16_t uvalue = uint16_t(float(current_mA) * (MAX17047_SENSE * 640.0f));
        value[0] = uvalue & 0xFF;
        value[1] = (uvalue>>8) & 0xFF;
        i2c.write(I2C_ADDRESS, MAX17047_I_CHG_TERM, value, 2);
    };

    // application specific (only set on POR)
    void setEmptyVoltage(const uint16_t empty_mV, const uint16_t recovery_mV) const
    {
        uint8_t  value[2];
        value[0] = (recovery_mV / 40) & 0b01111111;
        value[1] = (empty_mV / 20) & 0xFF;
        i2c.write(I2C_ADDRESS, MAX17047_V_EMPTY, value, 2);
    };

    // alarm turns the alarm-led on
    void setAlarmVoltage(const uint16_t lowVolt_mV, const uint16_t highVolt_mV) const
    {
        uint8_t  value[2];
        value[0] = (lowVolt_mV / 20) & 0xFF;       // low Voltage
        value[1] = (highVolt_mV / 20) & 0xFF; // high Voltage
        i2c.write(I2C_ADDRESS, MAX17047_V_ALRT_THRESHOLD, value, 2);
    };

    // do this at end-of-charge or end-of-discharge, gives back 20 byte, contains memory / INFO about cells
    void backupData(uint8_t registers[]) const
    {
        i2c.read(I2C_ADDRESS, MAX17047_FULL_CAP,       &registers[0], 2);
        i2c.read(I2C_ADDRESS, MAX17047_CYCLES,         &registers[2], 2);
        i2c.read(I2C_ADDRESS, MAX17047_RCOMP_0,        &registers[4], 2);
        i2c.read(I2C_ADDRESS, MAX17047_TEMP_CO,        &registers[6], 2);
        i2c.read(I2C_ADDRESS, MAX17047_Q_RESIDUAL_00,  &registers[8], 2);
        i2c.read(I2C_ADDRESS, MAX17047_Q_RESIDUAL_10,  &registers[10], 2);
        i2c.read(I2C_ADDRESS, MAX17047_Q_RESIDUAL_20,  &registers[12], 2);
        i2c.read(I2C_ADDRESS, MAX17047_Q_RESIDUAL_30,  &registers[14], 2);
        i2c.read(I2C_ADDRESS, MAX17047_D_QACC,         &registers[16], 2);
        i2c.read(I2C_ADDRESS, MAX17047_D_PACC,         &registers[18], 2);
    };

    // after POR you have can restore registers and don't have to relearn the cell
    void restoreData(uint8_t registers[]) const
    {
        // restore app-specific-register
        // restore gauge-learned register
        i2c.write(I2C_ADDRESS, MAX17047_FULL_CAP,      &registers[0], 2);
        i2c.write(I2C_ADDRESS, MAX17047_CYCLES,        &registers[2], 2);
        i2c.write(I2C_ADDRESS, MAX17047_RCOMP_0,       &registers[4], 2);
        i2c.write(I2C_ADDRESS, MAX17047_TEMP_CO,       &registers[6], 2);
        i2c.write(I2C_ADDRESS, MAX17047_Q_RESIDUAL_00, &registers[8], 2);
        i2c.write(I2C_ADDRESS, MAX17047_Q_RESIDUAL_10, &registers[10], 2);
        i2c.write(I2C_ADDRESS, MAX17047_Q_RESIDUAL_20, &registers[12], 2);
        i2c.write(I2C_ADDRESS, MAX17047_Q_RESIDUAL_30, &registers[14], 2);
        i2c.write(I2C_ADDRESS, MAX17047_D_QACC,        &registers[16], 2);
        i2c.write(I2C_ADDRESS, MAX17047_D_PACC,        &registers[18], 2);
    };

    // return measured cell voltage in mV
    uint16_t getCellVoltage_mV() const
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_V_CELL, value, 2);
        return uint16_t((uint16_t(value[1]<<5) + (value[0]>>3)) / 1.6); // mV
    };

    // return measured / drawn cell current in mA
    int16_t getCellCurrent_mA() const
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_CURRENT, value, 2);
        return (((value[1]<<8) + (value[0])) * (0.001 * (1.5625 / MAX17047_SENSE)));
    };

    // return cell current in mA
    int16_t getCellAverageCurrent_fmA() const
    {
		 uint8_t value[2];
		 i2c.read(I2C_ADDRESS, MAX17047_AVERAGE_CURRENT, value, 2);
		return float((value[1]<<8) + (value[0])) * (0.001 * (1.5625 / MAX17047_SENSE));
	};
	
    // State of Charge in percent
    float getStateOfCharge_f(void) const
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_SOC_AV, value, 2);
        return float(uint16_t(value[1]<<8) + (value[0])) / 256.0f;
    }

    // State of Charge in percent
    uint8_t getStateOfCharge(void) const
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_SOC_AV, value, 2);
        return (value[1]);
    }

    // returns 1 if cell is empty
    bool getEmptyStatus(void)  const
    {
        // empty-Status
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_F_STAT, value, 2);
        if (value[1] & 1)  return true;
        else               return false;
    }

    // gives remaining time in minutes
    float getTimeToEmpty_fmin(void) const
    {
        // time to empty
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_TTE, value, 2);
        return float(uint16_t(value[1]<<8) + (value[0])) * (5.625f / 60.0f);
    }

    // gives remaining time in minutes
    uint16_t getTimeToEmpty_min(void) const
    {
        // time to empty
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_TTE, value, 2);
        return (uint16_t(value[1]<<8) + (value[0])) / 11; // only an approximation to avoid float
    }

    // Remaining capacity in Ah
    uint16_t getRemainingCapacity_mAh(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_REM_CAP_AV, value, 2);
        const float fvalue = float(uint16_t(value[1]<<8) + (value[0])) * (0.005 / MAX17047_SENSE);
        return uint16_t(fvalue);
    }


    // full capacity
    uint16_t getFullCapacity_mAh(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_FULL_CAP, value, 2);
        const float fvalue = float(uint16_t(value[1] << 8) + (value[0])) * (0.005 / MAX17047_SENSE);
        return uint16_t(fvalue);
    }

    // MAX17047_CYCLES in percent
    uint16_t getChargingCycles_per(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_CYCLES, value, 2);
        return (uint16_t(value[1] << 8) + (value[0]));
    }

    // MAX17047_AGE in percent (float)
    float getCellAge_fper(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_AGE, value, 2);
        return float(uint16_t(value[1] << 8) + (value[0])) / 256.0f;
    }

    // MAX17047_AGE in percent (truncated int)
    uint16_t getCellAge_per(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_AGE, value, 2);
        return (value[1]);
    }

   // MAX17047_TEMPERATURE in Celcius
    float getTemperature_fc(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_TEMPERATURE, value, 2);
        return float(uint16_t(value[1] << 8) + (value[0])) / 256.0f;
    }

    // MAX17047_TEMPERATURE in Celcius
    uint16_t getTemperature_c(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_TEMPERATURE, value, 2);
        return (value[1]);
    }

    // MAX17047_DESIGN_CAP in microvolt hours (mAh capacity * sense resistor = microvolt hours
    float getDesignCap_fmAh(void)
    {
        uint8_t value[2];
        i2c.read(I2C_ADDRESS, MAX17047_DESIGN_CAP, value, 2);
        const float fvalue = float(uint16_t(value[1]<<8) + (value[0])) * (0.005 / MAX17047_SENSE);
        return (fvalue);
    }
    
    // return the instance's sense resistor value
    float getSense(void)
    {
        return (MAX17047_SENSE);
    }
    
    // set the instance's sense resistor value
    void setSense(const float& fvalue) {
        MAX17047_SENSE = fvalue;
    }
     /*
// feedback and status TODO: translate to getters:
void print_status() const
{
    uint8_t value[2];
    float   fvalue;

    i2c.read(I2C_ADDRESS, MAX17047_STATUS, value, 2);
    // Voltage Status
    if      (value[1] & 0b00000001)  Serial.print("|E"); // EMPTY
    else if (value[1] & 0b00010000)  Serial.print("|M"); // FULL exceed maximum
    else                             Serial.print("|N"); // normal

    // MAX17047_CURRENT
    i2c.read(I2C_ADDRESS, MAX17047_CURRENT, value, 2);
    fvalue = float((value[1]<<8) + (value[0])) * (0.001 * (1.5625 / MAX17047_SENSE));
    Serial.print(fvalue,2);
    Serial.print("|");
    i2c.read(I2C_ADDRESS, MAX17047_AVERAGE_CURRENT, value, 2);
    fvalue = float((value[1]<<8) + (value[0])) * (0.001 * (1.5625 / MAX17047_SENSE));
    Serial.print(fvalue,2);
    Serial.print(" mA :: ");
};
 */

};

#endif
