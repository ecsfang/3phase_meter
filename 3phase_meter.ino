#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include "SoftwareSerial.h"

#define WIFI_AP "YOUR_WIFI_AP"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance
EnergyMonitor emon2;                   // Create an instance
EnergyMonitor emon3;                   // Create an instance

#define I_CAL   (111.1/3.82)
#define VOLTAGE 220.0
#define SAMPLES 1480

#define RED_LED   10
#define GREEN_LED 11

int currentPins[3] = {0,1,2};              //Assign phase CT inputs to analog pins
double kilos[3];
unsigned long startMillis[3];
unsigned long endMillis[3];
double RMSCurrent[3];
int RMSPower[3];
int peakPower[3];

// Initialize the Ethernet client object
WiFiEspClient espClient;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

PubSubClient client(espClient);
int status = WL_IDLE_STATUS;

SoftwareSerial soft(0, 1); // RX, TX

void InitWiFi()
{
  // initialize serial for ESP module
  soft.begin(9600);
  // initialize ESP module
  WiFi.init(&soft);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network
  ConnectWifi();
}

void ConnectWifi() {
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Connecting to MQTT-server ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("House Power Meter", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}

void setup() 
{ 
  Serial.begin(115200);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  for(int i=0; i<=16;i++) {
    digitalWrite(RED_LED, i&0x01 ? HIGH : LOW);
    digitalWrite(GREEN_LED, i&0x02 ? HIGH : LOW);
    delay(200);
  }

  Serial.print("3 Phase Energy Meter");

  emon1.current(currentPins[0], I_CAL);             // Current: input pin, calibration.
  emon2.current(currentPins[1], I_CAL);             // Current: input pin, calibration.
  emon3.current(currentPins[2], I_CAL);             // Current: input pin, calibration.

  InitWiFi();
  client.setServer( mqttServer, 1883 );

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
  status = WiFi.status();
  if ( status != WL_CONNECTED)
    ConnectWifi();

  if ( !client.connected() )
    reconnect();

  readPhase();
  displayValues();
  client.publish( "theHouse/power/telemetry", "values" );

  delay(1000);

  client.loop();
}

void displayValues()
{
  static int lp=0;
  switch(lp++ % 4) {
    case 0:
      displayKilowattHours ();
      break;
    case 1:
      displayCurrent ();
      break;
    case 2:
      displayRMSPower ();
      break;
    case 3:
      displayPeakPower ();
      break;
  }
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
