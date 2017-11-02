#include <Wire.h>
#include <i2c.h> // iLib library
#include <i2c_MAX17047.h> // iLib library
MAX17047 max17047;

void setup()
{
  Serial.begin(9600);
  Serial.println("Probe MAX17047: ");
  if (max17047.getPORStatus()) {
    Serial.println("PORStatus:true");
    delay(600);
  } else {
    Serial.println("PORStatus:false");
  }
  Serial.print("sense resistor:");Serial.print(max17047.getSense());
  max17047.setSense(0.00016667);
}

void loop()
{

  if (max17047.getPORStatus()) {
    Serial.println("PORStatus:true");
    delay(600);
  } else {
    Serial.println("PORStatus:false");
  }
  Serial.print("cell voltage:"); Serial.print(max17047.getCellVoltage_mV()); Serial.println(" mV");
  Serial.print("cell current:"); Serial.print(max17047.getCellCurrent_mA()); Serial.println(" mA");
  Serial.print("cell average current:"); Serial.print(max17047.getCellAverageCurrent_fmA()); Serial.println(" mA");
  Serial.print("design capacity:"); Serial.print(max17047.getDesignCap_fmAh()); Serial.println(" mAh");
  Serial.print("full capacity:"); Serial.print(max17047.getFullCapacity_mAh()); Serial.println(" mAh");
  Serial.print("remaining capacity:"); Serial.print(max17047.getRemainingCapacity_mAh()); Serial.println(" mAh");
  Serial.print("time to empty:"); Serial.print(max17047.getTimeToEmpty_fmin()); Serial.println(" min");
  Serial.print("SOC:"); Serial.print(max17047.getStateOfCharge_f()); Serial.println("%");
  Serial.print("Cell Age (SOH):"); Serial.print(max17047.getCellAge_fper()); Serial.println("%");
  Serial.print("cycles:"); Serial.print(max17047.getChargingCycles_per()); Serial.println("%");
  Serial.print("temperature:");Serial.print(max17047.getTemperature_fc());Serial.println(" C");
  Serial.println("");
  delay(3000);

}

