// I2C
#include <Wire.h>
#include "i2c.h"

#include "i2c_MPL3115A2.h"
MPL3115A2 mpl3115 = MPL3115A2();

void setup()
{
  Serial.begin(115200);
  Serial.println("READ MPL3115A2");

  if (mpl3115.initialize()) { Serial.println("Sensor found"); }
  else                      { Serial.println("Sensor missing"); while (1) {} }

  // onetime-measure:
  mpl3115.setEnabled(1);
  mpl3115.startMeasurement();
}

void loop() {
  mpl3115.waitMeasurement();

  float altitude;
  mpl3115.getAltitude(altitude);

  float temperature;
  mpl3115.getTemperature(temperature);
  mpl3115.startMeasurement();

  Serial.print(" Height: ");
  Serial.print(altitude);
  Serial.print(" Temp: ");
  Serial.print(temperature);
  Serial.println("");
}


