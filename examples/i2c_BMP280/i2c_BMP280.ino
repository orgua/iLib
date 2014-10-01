#include <Wire.h>
#include "i2c.h"

#include "i2c_BMP280.h"
BMP280 bmp280 = BMP280();

void setup()
{
    Serial.begin(115200);
    Serial.print("Probe BMP280: ");

    if (bmp280.initialize()) Serial.println("Sensor found");
    else
    {
        Serial.println("Sensor missing");
        while (1) {}
    }

    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
}

void loop()
{
    bmp280.awaitMeasurement();

    float pascal;
    bmp280.getPressure(pascal);

    float temperature;
    bmp280.getTemperature(temperature);
    bmp280.triggerMeasurement();

    Serial.print(" Pressure: ");
    Serial.print(pascal);
    Serial.print(" Temp: ");
    Serial.print(temperature);
    Serial.println("");
}

/**<

Program size:
A1.0.5:
A1.5.7: 6980b

 */

