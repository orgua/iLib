#include <SPI.h>
#include "spi_rfm95.h"
RFM95 rfm = RFM95();


void setup()
{
    Serial.begin(115200);

    Serial.print("Probe RFM95W: ");
    if (rfm.initialize()) Serial.println("missing");
    else Serial.println("found");

    while (1) {};
}

void loop()
{
    Serial.print(" test ");
    delay(50);
}

/**<

Program size:
A105: b
A157: b

 */
