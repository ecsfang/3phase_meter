#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance
EnergyMonitor emon2;                   // Create an instance
EnergyMonitor emon3;                   // Create an instance

#define I_CAL   (111.1/3.82)
#define VOLTAGE 220.0
#define SAMPLES 1480

int currentPins[3] = {0,1,2};              //Assign phase CT inputs to analog pins
double kilos[3];
unsigned long startMillis[3];
unsigned long endMillis[3];
double RMSCurrent[3];
int RMSPower[3];
int peakPower[3];

void setup() 
{ 
  Serial.begin(115200);
  Serial.print("3 Phase Energy Meter");

  emon1.current(currentPins[0], I_CAL);             // Current: input pin, calibration.
  emon2.current(currentPins[1], I_CAL);             // Current: input pin, calibration.
  emon3.current(currentPins[2], I_CAL);             // Current: input pin, calibration.

  delay(1000);
}

// Return true if all sensors are idle, false otherwise
int checkIdle ()
{
  int n = 0;
  for(int i=0;i<=2;i++)
    if( 511 != analogRead(currentPins[i]) )
      return false;
  return true;
}  

void readPhase ()      //Method to read information from CTs
{
  memset(RMSCurrent, 0, 3*sizeof(double));
  if( !checkIdle() ) {
    RMSCurrent[0] = emon1.calcIrms(SAMPLES);  // Calculate Irms only
    RMSCurrent[1] = emon2.calcIrms(SAMPLES);  // Calculate Irms only
    RMSCurrent[2] = emon3.calcIrms(SAMPLES);  // Calculate Irms only
  }
  
  for(int i=0;i<=2;i++)
  {
    RMSPower[i] = VOLTAGE*RMSCurrent[i];    //Calculates RMS Power Assuming Voltage 220VAC, change to 110VAC accordingly
    if (RMSPower[i] > peakPower[i])
    {
      peakPower[i] = RMSPower[i];
    }
    endMillis[i]= millis();
    unsigned long time = (endMillis[i] - startMillis[i]);
    kilos[i] = kilos[i] + (RMSPower[i] * (time/60/60/1000000));    //Calculate kilowatt hours used
    startMillis[i]= millis();
  }
}

void loop()   //Calls the methods to read values from CTs and changes display
{
  readPhase();
  displayKilowattHours ();
  delay(1000);
  readPhase();
  displayCurrent ();
  delay(1000);
  readPhase();
  displayRMSPower ();
  delay(1000);
  readPhase();
  displayPeakPower ();
  delay(1000);
}

void displayKilowattHours ()  //Displays all kilowatt hours data
{
  Serial.print(kilos[0]);
  Serial.print("kWh ");
  Serial.print(kilos[1]);
  Serial.print("kWh ");
  Serial.print(kilos[2]);
  Serial.print("kWh ");
  Serial.println("Energy");
}

void displayCurrent ()      //Displays all current data
{
  Serial.print(RMSCurrent[0]);
  Serial.print("A ");
  Serial.print(RMSCurrent[1]);
  Serial.print("A ");
  Serial.print(RMSCurrent[2]);
  Serial.print("A ");
  Serial.println("Current");
}

void displayRMSPower ()     //Displays all RMS power data
{
  Serial.print(RMSPower[0]);
  Serial.print("W ");
  Serial.print(RMSPower[1]);
  Serial.print("W ");
  Serial.print(RMSPower[2]);
  Serial.print("W ");
  Serial.println("Power");
}

void displayPeakPower ()    //Displays all peak power data
{
  Serial.print(peakPower[0]);
  Serial.print("W ");
  Serial.print(peakPower[1]);
  Serial.print("W ");
  Serial.print(peakPower[2]);
  Serial.print("W ");
  Serial.println("Max Pwr");
}

#if 0



// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance
EnergyMonitor emon2;                   // Create an instance
EnergyMonitor emon3;                   // Create an instance

#define I_CAL   (111.1/3.82)
#define VOLTAGE 220.0
#define SAMPLES 1480
void setup()
{  
  Serial.begin(9600);
  
  emon1.current(0, I_CAL);             // Current: input pin, calibration.
  emon2.current(1, I_CAL);             // Current: input pin, calibration.
  emon3.current(2, I_CAL);             // Current: input pin, calibration.
}

void loop()
{
  double Irms1 = emon1.calcIrms(SAMPLES);  // Calculate Irms only
  double Irms2 = emon2.calcIrms(SAMPLES);  // Calculate Irms only
  double Irms3 = emon3.calcIrms(SAMPLES);  // Calculate Irms only
  
  Serial.print(Irms1*VOLTAGE);         // Apparent power
  Serial.print(" ");
  Serial.print(Irms1);          // Irms
  Serial.print(" | ");
  Serial.print(Irms2*VOLTAGE);         // Apparent power
  Serial.print(" ");
  Serial.print(Irms2);          // Irms
  Serial.print(" | ");
  Serial.print(Irms3*VOLTAGE);         // Apparent power
  Serial.print(" ");
  Serial.println(Irms3);          // Irms
  Serial.print("Vcc = ");
  Serial.println(emon1.readVcc());
}

#endif
