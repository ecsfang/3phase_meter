// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance
EnergyMonitor emon2;                   // Create an instance
EnergyMonitor emon3;                   // Create an instance

void setup()
{  
  Serial.begin(9600);
  
  emon1.current(0, 111.1);             // Current: input pin, calibration.
  emon2.current(1, 111.1);             // Current: input pin, calibration.
  emon3.current(2, 111.1);             // Current: input pin, calibration.
}

void loop()
{
  double Irms1 = emon1.calcIrms(1480);  // Calculate Irms only
  double Irms2 = emon2.calcIrms(1480);  // Calculate Irms only
  double Irms3 = emon3.calcIrms(1480);  // Calculate Irms only
  
  Serial.print(Irms1*230.0);         // Apparent power
  Serial.print(" ");
  Serial.print(Irms1);          // Irms
  Serial.print(" | ");
  Serial.print(Irms2*230.0);         // Apparent power
  Serial.print(" ");
  Serial.print(Irms2);          // Irms
  Serial.print(" | ");
  Serial.print(Irms3*230.0);         // Apparent power
  Serial.print(" ");
  Serial.println(Irms3);          // Irms
}
