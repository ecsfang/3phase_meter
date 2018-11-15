/***************************************************************************
  POWER METER - with 3 phases and blink detection
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "3phase_adc.h"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include "mySSID.h"
#include "EmonLib.h"

const int sclPin =            D1;
const int sdaPin =            D2;
const int blinkPin =          D0;


bool bHaveADS = false;

extern ADS1115 adc;

// Create  instances for each CT channel
EnergyMonitor ct[4]; //ct1, ct2, ct3, ct4;

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_LEN 50
char msg[MSG_LEN];

unsigned long delayTime = 1000;


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("PowerMeter!"));

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(blinkPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(blinkPin), onPulse, FALLING);

  for(int i=0; i<16; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // Off ...
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  // Off ...
    delay(100);
  }

  bool status;

  Wire.begin(sdaPin, sclPin);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  i2cScan();

  if ( bHaveADS ) {
    initAdc(adc);
  }

  Serial.println();

  for (int i = 0; i < 3; i++)
    ct[i].inputPinReader = ads1115PinReader; // Replace the default pin reader with the customized ads pin reader

  // Calibration factor = CT ratio / burden resistance
  // the current constant is the value of current you want to
  // read when 1 V is produced at the analogue input
  ct[0].current(ADS0_A0, 30);
  ct[1].current(ADS0_A1, 30);
  ct[2].current(ADS0_A2, 30);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
#ifdef RX_DEBUG
    Serial.print("Attempting another MQTT connection...");
#endif
    // Attempt to connect
    if (client.connect("thePowerMeter")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("powerMeter", "ready");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

bool bBlink = false;
int blinkCnt = 0;
int tCnt = 0;

void loop()
{
  digitalWrite(LED_BUILTIN, LOW);  // On ...
  if (!client.connected()) {
    reconnect();
  }

  if (client.connected()) {
    client.loop();
  }

  ArduinoOTA.handle();

  if ( bBlink ) {
    Serial.print("Blinks: ");
    Serial.println(blinkCnt);
    bBlink = false;
  }
  tCnt++;
  if ( bHaveADS )
    read3Phase();
  digitalWrite(LED_BUILTIN, HIGH);  // Off ...
  delay(delayTime);
}

// The interrupt routine
void onPulse()
{
  digitalWrite(LED_BUILTIN, LOW);
  blinkCnt++;
  bBlink = true;
}

void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      switch ( address ) {
        case ADS1115_ADDRESS_ADDR_GND:
          Serial.print(" [ADS]");
          bHaveADS = true;
          break;
      }
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.print("Done! ");
    Serial.print(nDevices);
    Serial.println(" devices found.\n");
  }

  delay(2000);
}

void sendMsg(const char *topic, const char *m)
{
  Serial.print("Publish message: ");
  snprintf (msg, MSG_LEN, "powermeter/%s", topic);
  Serial.print(msg);
  Serial.print(" ");
  Serial.println(m);
  client.publish(msg, m);
}

void sendMsgF(const char *topic, double v)
{
  char buf[32];
  snprintf (buf, 32, "%.2f", v);
  sendMsg(topic, buf);
}

unsigned long startMillis = 0;
unsigned long endMillis = 0;

// The ADC input range (or gain) can be changed via the following
// functions, but be careful never to exceed VDD +0.3V max, or to
// exceed the upper and lower limits if you adjust the input range!
// Setting these values incorrectly may destroy your ADC!
//                                                                ADS1015  ADS1115
//                                                                -------  -------
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

#define DELTA_AMP 0.1

// Return true if change is big enough to report
bool change(double a, double b)
{
  return (a > b ? (a - b) : (b - a)) > DELTA_AMP;
}

void read3Phase(void)
{
  double  irms;
  double  oldIrms[3] = { -99, -99, -99};
  char    sensor[16];

  digitalWrite(LED_BUILTIN, LOW);

  for ( int c = 0; c < 3; c++) {
    irms = ct[c].calcIrms(1480);
    if ( change(irms, oldIrms[c] ) ) {
      snprintf(sensor, 16, "phase%d", c + 1);
      sendMsgF(sensor, irms);
      oldIrms[c] = irms;
    }
  }

  digitalWrite(LED_BUILTIN, HIGH);
}
/*
  void checkADC(void)
  {
  int16_t adc0, adc1, adc2, adc3;
  double  irms;

  digitalWrite(LED_BUILTIN, LOW);

  adc0 = ads1115PinReader(ADS0_A0);
  adc1 = ads1115PinReader(ADS0_A1);
  adc2 = ads1115PinReader(ADS0_A2);
  adc3 = ads1115PinReader(ADS0_A3);
  Serial.print("AIN0: "); Serial.print(adc0);
  Serial.print(" AIN1: "); Serial.print(adc1);
  Serial.print(" AIN2: "); Serial.print(adc2);
  Serial.print(" AIN3: "); Serial.println(adc3);

  Serial.print("ct1: ");
  startMillis = millis();
  irms = ct1.calcIrms(1480);
  sendMsgF("phase1", irms);
  Serial.print( irms );
  endMillis = millis() - startMillis;
  Serial.print(" - "); Serial.print(endMillis); Serial.println(" ms");

  Serial.print("ct2: ");
  startMillis = millis();
  irms = ct2.calcIrms(1480);
  sendMsgF("phase2", irms);
  Serial.print( irms );
  endMillis = millis() - startMillis;
  Serial.print(" - "); Serial.print(endMillis); Serial.println(" ms");

  Serial.print("ct3: ");
  startMillis = millis();
  irms = ct3.calcIrms(1480);
  sendMsgF("phase3", irms);
  Serial.print( irms );
  endMillis = millis() - startMillis;
  Serial.print(" - "); Serial.print(endMillis); Serial.println(" ms");

  Serial.println(" ");
  digitalWrite(LED_BUILTIN, HIGH);
  }
**/
