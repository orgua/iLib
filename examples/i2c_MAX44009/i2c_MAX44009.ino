// I2C
#include <Wire.h>
#include "i2c.h"
#include "i2c_MAX44009.h"
MAX44009 max44009 = MAX44009();


void setup()
{
  Serial.begin(115200);
  Serial.println("READ MAX44009");

  if (max44009.initialize()) { Serial.println("Sensor found"); } 
  else                       { Serial.println("Sensor missing"); while (1) { }; }
}

void loop() { 
  static unsigned long mLux_value;

  max44009.getValue(mLux_value);

  Serial.print("mLUX: ");
  Serial.print(mLux_value);
  Serial.println(" ");

  delay(40);
}


