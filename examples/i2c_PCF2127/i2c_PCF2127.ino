#include <Wire.h>
#include "i2c.h"

#include "i2c_PCF2127.h"
PCF2127 pcf2127 = PCF2127();


void setup()
{
    Serial.begin(115200);

    Serial.print("Probe PCF2127: ");
    if (pcf2127.initialize()) Serial.println("Module found");
    else
    {
        Serial.println("Module missing");
        while (1) {}
    }

    pcf2127.setYears(14);
    pcf2127.setMonth(10);
    pcf2127.setDays(3);

    char time = '__TIME__';

    pcf2127.setHours(10);
    pcf2127.setMinutes(10);
    pcf2127.setSeconds(10);
}

void loop()
{
    uint8_t Y,M,D,h,m,s;

    pcf2127.setYears(Y);
    pcf2127.setMonth(M);
    pcf2127.setDays(D);
    pcf2127.setHours(h);
    pcf2127.setMinutes(m);
    pcf2127.setSeconds(s);

    Serial.print(2000+Y);
    Serial.print("-");
    Serial.print(M);
    Serial.print("-");
    Serial.print(D);
    Serial.print("-");

    Serial.print(h);
    Serial.print(":");
     Serial.print(m);
    Serial.print(":");
        Serial.print(s);
    Serial.println("");
    delay(200);
}

/**<

Program size:
A1.0.5: 6754b
A1.5.7: 6454b

 */
