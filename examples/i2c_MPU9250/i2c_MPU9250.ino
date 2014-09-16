#include <Wire.h>
#include "i2c.h"

#include "i2c_MPU9250.h"
MPU9250 mpu9250 = MPU9250();

void setup()
{
    Serial.begin(115200);
    Serial.println("READ MPU9250");

    if (mpu9250.initialize()) Serial.println("Sensor found!");
    else
    {
        Serial.println("Sensor missing");
        while(1) {};
    }
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
    Serial.println("");
    delay(20);
}

/**<

Program size:
A1.0.5:
A1.5.7:

 */
