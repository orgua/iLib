#include <Wire.h>
#include "i2c.h"

#include "i2c_MPU9250.h"
MPU9250 mpu9250 = MPU9250();

void setup()
{
    Serial.begin(115200);
    Serial.println("READ MPU9250");

    if (mpu9250.initialize()) Serial.println("MPU9250 found!");
    else
    {
        Serial.println("MPU9250 missing");
        while(1) {};
    }

    if (i2c.probe(0x0C)) Serial.println("AK8963 found!");
    else                 Serial.println("AK8963 missing");


    Serial.println("Scan I2C-Bus for responses");
    uint8_t address, result;
    for(address = 0; address < 128; address++ )
    {
        result = i2c.probe(address);
        if (result)
        {
            Serial.print("Found: 0x");
            if (address < 17) Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("");
        }
        delay(20);
    }
    Serial.println("");
    Serial.println("DONE");

}

void loop()
{
    static float xyz_GyrAccMag[9];

    mpu9250.getMeasurement(xyz_GyrAccMag);

    Serial.print(" X: ");
    Serial.print(xyz_GyrAccMag[0],2);
    Serial.print(" \tY: ");
    Serial.print(xyz_GyrAccMag[1],2);
    Serial.print(" \tZ: ");
    Serial.print(xyz_GyrAccMag[2],2);

    Serial.print(" \t Temp: ");
    Serial.print(xyz_GyrAccMag[3],2);

    Serial.println("");
    delay(20);
}

/**<

Program size:
A1.0.5:
A1.5.7:

 */
