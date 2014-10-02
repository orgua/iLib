#ifndef i2c_pcf2127_h
#define i2c_pcf2127_h

#include "i2c.h"



/** ######################################################################

Driver for the PCF2127T RTC

 CONSUMPTION: standby X µA, measure X µA

 Details:


########################################################################  */

class PCF2127
{

    /** ######### Register-Map ################################################################# */

#define PCF_ADDRESS 	        0x51

#define REG_CONTROL1            0x00
#define     MSK_CONTROL1_EXTTST (B10000000) // test mode? NO
#define     MSK_CONTROL1_T      (B01000000) // unused
#define     MSK_CONTROL1_STOP   (B00100000)
#define     MSK_CONTROL1_TSF1   (B00010000)
#define     MSK_CONTROL1_POROVRD (B00001000)
#define     MSK_CONTROL1_12_24  (B00000100)
#define     MSK_CONTROL1_MI     (B00000010)
#define     MSK_CONTROL1_SI     (B00000001)

#define REG_CONTROL2            0x01
#define     MSK_CONTROL2_MSF    (B10000000)
#define     MSK_CONTROL2_WDTF   (B01000000)
#define     MSK_CONTROL2_TSF2   (B00100000)
#define     MSK_CONTROL2_AF     (B00010000)
#define     MSK_CONTROL2_CDTF   (B00001000)
#define     MSK_CONTROL2_TSIE   (B00000100)
#define     MSK_CONTROL2_AIE    (B00000010)
#define     MSK_CONTROL2_CDTIE  (B00000001)
#define REG_CONTROL3            0x02
#define     MSK_CONTROL3_PWRMNG (B11100000)
#define     MSK_CONTROL3_BTSE   (B00010000)
#define     MSK_CONTROL3_BF     (B00001000)
#define     MSK_CONTROL3_BLF    (B00000100)
#define     MSK_CONTROL3_BIE    (B00000010)
#define     MSK_CONTROL3_BLIE   (B00000001)

#define REG_SECONDS             0x03    // 7bit
#define REG_MINUTES             0x04    // 7bit
#define REG_HOURS               0x05    // 6bit
#define REG_DAYS                0x06    // 6bit
#define REG_WEEKDAYS            0x07    // 3bit
#define REG_MONTH               0x08    // 5bit
#define REG_YEARS               0x09    // 8bit

#define REG_SECOND_ALARM        0x0A
#define REG_MINUTE_ALARM        0x0B
#define REG_HOUR_ALARM          0x0C
#define REG_DAY_ALARM           0x0D
#define REG_WEEKDAY_ALARM       0x0E
#define     VAL_ENABLE_ALARM    (B10000000) // highest bit always for Enable!

#define REG_CLOCKOUT_CTL        0x0F
#define     MSK_CLOCKOUT_TCR    (B11000000)
#define     MSK_CLOCKOUT_COF    (B00000111)

#define REG_WATCHDOG_TIM_CTL    0x10
#define     MSK_WATCHDOG_WDCD   (B11000000)
#define     MSK_WATCHDOG_TITP   (B00100000)
#define     MSK_WATCHDOG_TF     (B00000011)
#define REG_WATCHDOG_TIM_VAL    0x11

#define REG_TIMESTP_CTL         0x12
#define     MSK_TIMESTP_TSM     (B10000000)
#define     MSK_TIMESTP_TSOFF   (B01000000)
#define     MSK_TIMESTP_1O16    (B00011111)
#define REG_SEC_TIMESTP         0x13
#define REG_MIN_TIMESTP         0x14
#define REG_HOUR_TIMESTP        0x15
#define REG_DAY_TIMESTP         0x16
#define REG_MON_TIMESTP         0x17
#define REG_YEAR_TIMESTP        0x18

#define REG_AGING_OFFSET        0x19    // 4bit

#define REG_RAM_ADDR_MSB        0x1A    // 1bit / last
#define REG_RAM_ADDR_LSB        0x1B
#define REG_RAM_WRT_CMD         0x1C
#define REG_RAM_RD_CMD          0x1D
    /** ######### function definition ################################################################# */

public:
    /**< TODO: do i need a constructor? */
    PCF2127(void)
    {
        //_address = MPL_ADDRESS;
    };

    uint8_t initialize()
    {
        if (i2c.probe(PCF_ADDRESS)==0) return 0;

        uint8_t regPCF;

        regPCF  = 0;
        regPCF &= !MSK_CONTROL1_EXTTST;
        regPCF &= !MSK_CONTROL1_STOP;
        regPCF &= !MSK_CONTROL1_TSF1;
        regPCF &= !MSK_CONTROL1_POROVRD;
        regPCF &= !MSK_CONTROL1_12_24;
        regPCF &= !MSK_CONTROL1_MI;
        regPCF &= !MSK_CONTROL1_SI;
        i2c.writeByte(PCF_ADDRESS, REG_CONTROL1, regPCF);

        regPCF  = 0;
        regPCF &= !MSK_CONTROL2_TSIE;
        regPCF &= !MSK_CONTROL2_AIE;
        regPCF &= !MSK_CONTROL2_CDTIE;
        i2c.writeByte(PCF_ADDRESS, REG_CONTROL2, regPCF);

        regPCF  = 0;
        regPCF |= MSK_CONTROL3_PWRMNG & B10100000; // direct switching and no low power detection
        regPCF &= !MSK_CONTROL3_BTSE;
        regPCF &= !MSK_CONTROL3_BIE;
        regPCF &= !MSK_CONTROL3_BLIE;
        i2c.writeByte(PCF_ADDRESS, REG_CONTROL3, regPCF);


        setTemperaturePeriod(240);

        return 1;
    };

    void clearInterrupt()
    {
        i2c.setRegister(PCF_ADDRESS, REG_CONTROL1, MSK_CONTROL1_TSF1, 0);
    }

    void setTemperaturePeriod(uint8_t seconds=240)
    {
        if      (seconds > 160)  seconds = B00000000;    // 240s
        else if (seconds > 90)   seconds = B01000000;    // 120s
        else if (seconds > 45)   seconds = B10000000;    // 60s
        else                     seconds = B11000000;    // 30s
        i2c.setRegister(PCF_ADDRESS, REG_CLOCKOUT_CTL, MSK_CLOCKOUT_TCR, seconds);
    }

    void setClockOut(uint16_t clock = 1)
    {
        if      (clock > 30000) clock = 000;    // 32768
        else if (clock > 16000) clock = 001;    // 16384
        else if (clock > 8000)  clock = 010;    // 8192
        else if (clock > 4000)  clock = 011;    // 4096
        else if (clock > 2000)  clock = 100;    // 2048
        else if (clock > 1000)  clock = 101;    // 1024
        else if (clock > 0)     clock = 110;    // PPS
        else                    clock = 111;    // disable
        i2c.setRegister(PCF_ADDRESS, REG_CLOCKOUT_CTL, MSK_CLOCKOUT_COF, clock);

    }

    void setSeconds(uint8_t unit = 0)
    {
        if (unit > 60) return;
        uint8_t decimal = 0;
        while (unit > 9)
        {
            unit -= 10;
            decimal++;
        }
        unit |= (decimal)<<4;
        i2c.writeByte(PCF_ADDRESS, REG_SECONDS, unit);
    }

    void getSeconds(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_SECONDS);
        unit = ((unit>>4) * 10) + (unit &= B00001111);
    }

    void setMinutes(uint8_t unit = 0)
    {
        if (unit > 60) return;
        uint8_t decimal = 0;
        while (unit > 9)
        {
            unit -= 10;
            decimal++;
        }
        unit |= (decimal)<<4;
        i2c.writeByte(PCF_ADDRESS, REG_MINUTES, unit);
    }

    void getMinutes(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_MINUTES);
        unit = ((unit>>4) * 10) + (unit &= B00001111);
    }

    void setHours(uint8_t unit = 0)
    {
        if (unit > 24) return;
        uint8_t decimal = 0;
        while (unit > 9)
        {
            unit -= 10;
            decimal++;
        }
        unit |= (decimal)<<4;
        i2c.writeByte(PCF_ADDRESS, REG_HOURS, unit);
    }

    void getHours(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_HOURS);
        unit = ((unit>>4) * 10) + (unit &= B00001111);
    }

    void setDays(uint8_t unit = 0)
    {
        if (unit > 32) return;
        uint8_t decimal = 0;
        while (unit > 9)
        {
            unit -= 10;
            decimal++;
        }
        unit |= (decimal)<<4;
        i2c.writeByte(PCF_ADDRESS, REG_DAYS, unit);
    }

     void getDays(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_DAYS);
        unit = ((unit>>4) * 10) + (unit &= B00001111);
    }

    void setWeekdays(uint8_t unit = 0)
    {
        if (unit > 6) return;
        i2c.writeByte(PCF_ADDRESS, REG_WEEKDAYS, unit); // start on Sunday
    }

         void getWeekdays(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_WEEKDAYS);
    }

    void setMonth(uint8_t unit = 0)
    {
        if (unit > 12) return;
        uint8_t decimal = 0;
        while (unit > 9)
        {
            unit -= 10;
            decimal++;
        }
        unit |= (decimal)<<4;
        i2c.writeByte(PCF_ADDRESS, REG_MONTH, unit);
    }

    void getMonth(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_MONTH);
        unit = ((unit>>4) * 10) + (unit &= B00001111);
    }

    void setYears(uint8_t unit = 0)
    {
        if (unit > 12) return;
        uint8_t decimal = 0;
        while (unit > 9)
        {
            unit -= 10;
            decimal += 1;
        }
        unit |= (decimal)<<4;
        i2c.writeByte(PCF_ADDRESS, REG_MONTH, unit);
    }

    void getYears(uint8_t& unit)
    {
        unit = i2c.readByte(PCF_ADDRESS, REG_MONTH);
        unit = ((unit>>4) * 10) + (unit &= B00001111);
    }
/*

#define REG_SECONDS             0x03    // 7bit
#define REG_MINUTES             0x04    // 7bit
#define REG_HOURS               0x05    // 6bit
#define REG_DAYS                0x06    // 6bit
#define REG_WEEKDAYS            0x07    // 3bit
#define REG_MONTH               0x08    // 5bit
#define REG_YEARS               0x09    // 8bit

#define REG_SECOND_ALARM        0x0A
#define REG_MINUTE_ALARM        0x0B
#define REG_HOUR_ALARM          0x0C
#define REG_DAY_ALARM           0x0D
#define REG_WEEKDAY_ALARM       0x0E
#define     VAL_ENABLE_ALARM    (B10000000) // highest bit always for Enable!



#define REG_WATCHDOG_TIM_CTL    0x10
#define     MSK_WATCHDOG_WDCD   (B11000000)
#define     MSK_WATCHDOG_TITP   (B00100000)
#define     MSK_WATCHDOG_TF     (B00000011)
#define REG_WATCHDOG_TIM_VAL    0x11

#define REG_AGING_OFFSET        0x19    // 4bit

#define REG_RAM_ADDR_MSB        0x1A    // 1bit / last
#define REG_RAM_ADDR_LSB        0x1B
#define REG_RAM_WRT_CMD         0x1C
#define REG_RAM_RD_CMD          0x1D
*/
};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//PCF2127 pcf2127 = PCF2127();

#endif
