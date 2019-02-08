/***************************************************************************
  POWER METER - with 3 phases and blink detection
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
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

//extern ADS1115 adc;

//#define USE_MQTT
#define NR_OF_PHASES  3

// Create  instances for each CT channel
EnergyMonitor ct[NR_OF_PHASES];

int sctPin[NR_OF_PHASES] = { ADS_A0, ADS_A1, ADS_A2 };

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_LEN 50
char msg[MSG_LEN];

unsigned long delayTime = 1000;

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println(F("PowerMeter!"));

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(blinkPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(blinkPin), onPulse, FALLING);

  for(int i=0; i<16; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // On ...
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  // Off ...
    delay(100);
  }

  Wire.begin(sdaPin, sclPin);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

#ifdef USE_ADS1015
  i2cScan();

  if ( bHaveADS ) {
    initAdc();
  }
#else
  initAdc();
#endif

  Serial.println();

  // Calibration factor = CT ratio / burden resistance
  // the current constant is the value of current you want to
  // read when 1 V is produced at the analogue input
  for (int i = 0; i < NR_OF_PHASES; i++) {
    ct[i].inputPinReader = adcPinReader; // Replace the default pin reader with the customized ads pin reader
    ct[i].current(sctPin[i], 30);
  }

#ifdef USE_ADS1015
  ArduinoOTA.setHostname("PowerMeterADS");
#endif
#ifdef USE_MCP3008
  ArduinoOTA.setHostname("PowerMeterMCP");
#endif
  ArduinoOTA.setPassword(flashpw);

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
    } else {
      Serial.println("Unknown error!");
    }
  });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#ifdef USE_MQTT
  client.setServer(mqtt_server, 1883);
#endif
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
#ifdef RX_DEBUG
    Serial.print("Attempting another MQTT connection...");
#endif
    // Attempt to connect
    if (client.connect("thePowerMeter")) {
      Serial.println("Connected!");
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
#ifdef USE_MQTT
  if (!client.connected())
    reconnect();

  if (client.connected())
    client.loop();
#endif

  ArduinoOTA.handle();

  if ( bBlink ) {
    Serial.print("Blinks: ");
    Serial.println(blinkCnt);
    bBlink = false;
  }

//  testADC();
  read3Phase();
  sendStatus();

  delay(delayTime);
  tCnt++;
}

// The interrupt routine
void onPulse()
{
  blinkCnt++;
  bBlink = true;
}

#ifdef USE_ADS1015
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
#endif

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
void sendMsgI(const char *topic, int v)
{
  char buf[32];
  snprintf (buf, 32, "%d", v);
  sendMsg(topic, buf);
}

//unsigned long startMillis = 0;
//unsigned long endMillis = 0;

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
#define DELTA_POW 50

// Return true if change is big enough to report
bool change(double a, double b, double diff)
{
  return (a > b ? (a - b) : (b - a)) > diff;
}

// Housekeeping ....
unsigned long startMillis[NR_OF_PHASES];
unsigned long endMillis[NR_OF_PHASES];
double        oldIrms[NR_OF_PHASES] = { -99, -99, -99};
int           oldPower[NR_OF_PHASES] = { -99, -99, -99};

// Current values ...
double        irms[NR_OF_PHASES];
unsigned long RMSPower[NR_OF_PHASES];       // Current power (W)
unsigned long peakPower[NR_OF_PHASES];      // Peak power (per day)
double        kilos[NR_OF_PHASES];          // Total kWh today (per phase)
unsigned long todayPower;                   // Todays total
unsigned long yesterdayPower;               // Yesterdays total

void runAtMidnight(void)
{
  yesterdayPower = 0;
  for ( int c = 0; c < NR_OF_PHASES; c++) {
    yesterdayPower += (unsigned long)kilos[c];
    kilos[c] = 0.0;
    todayPower = 0;
    peakPower[c] = 0;
  }
}

#define SENSOR_PAR_CNT (NR_OF_PHASES*4+2)
#define BUF_LEN 1024

void sendStatus(void)
{
  char    values[BUF_LEN];
  int     n = 0;

  n = sprintf(values, "VALUES:%ld;", todayPower);
  n += sprintf(values+n, "%ld;", yesterdayPower);
  for( int c=0; c<NR_OF_PHASES; c++ )
    n += sprintf(values+n, "%.1f;", irms[c]);
  for( int c=0; c<NR_OF_PHASES; c++ )
    n += sprintf(values+n, "%d;", RMSPower[c]);
  for( int c=0; c<NR_OF_PHASES; c++ )
    n += sprintf(values+n, "%.1f;", kilos[c]);
  for( int c=0; c<NR_OF_PHASES; c++ )
    n += sprintf(values+n, "%d;", peakPower[c]);

  char *par[SENSOR_PAR_CNT];
  uint8_t cnt = 0;
  char *p = strstr(values, ":");
  while (p && cnt < SENSOR_PAR_CNT) {
    *(p++) = 0;
    par[cnt++] = p;
    p = strchr(p, ';');
  }
  
  // Fill in report in GAUS format
  String json;
  json = R"(
    {
      "Time": $TIME,
      "ENERGY": {
        "Total": $TOTAL,
        "Yesterday":$YDAY,
        "Today1": $TODAY_1,
        "Today2": $TODAY_2,
        "Today3": $TODAY_3,
        "Current1": $CURRENT_1,
        "Current2": $CURRENT_2,
        "Current3": $CURRENT_3,
        "Power1": $POWER_1,
        "Power2": $POWER_2,
        "Power3": $POWER_3,
        "Peak1": $PEAK_1,
        "Peak2": $PEAK_2,
        "Peak3": $PEAK_3
        }
    }
  )";
  json.replace("$TIME", "2018-11-27");
  json.replace("$TOTAL",      par[0]);
  json.replace("$YDAY",       par[1]);
  json.replace("$CURRENT_1",  par[2]);
  json.replace("$CURRENT_2",  par[3]);
  json.replace("$CURRENT_3",  par[4]);
  json.replace("$POWER_1",    par[5]);
  json.replace("$POWER_2",    par[6]);
  json.replace("$POWER_3",    par[7]);
  json.replace("$TODAY_1",    par[8]);
  json.replace("$TODAY_2",    par[9]);
  json.replace("$TODAY_3",    par[10]);
  json.replace("$PEAK_1",     par[11]);
  json.replace("$PEAK_2",     par[12]);
  json.replace("$PEAK_3",     par[13]);

  Serial.println( json );
}


void read3Phase(void)
{
  digitalWrite(LED_BUILTIN, LOW);

  for ( int c = 0; c < NR_OF_PHASES; c++) {
    irms[c] = ct[c].calcIrms(1480);
    RMSPower[c] = 230*irms[c];
    if( RMSPower[c] > peakPower[c] )
      peakPower[c] = RMSPower[c];
    // Get time since last reading ...
    endMillis[c] = millis();
    unsigned long time = (endMillis[c] - startMillis[c]);
    startMillis[c] = endMillis[c];
    // How much power has been used ... ?
    double duration = ((double)time)/(3600000.0); // Time in hours since last reading
    kilos[c] += RMSPower[c] * duration / 1000; // So many kWh have been used ...
  }
  digitalWrite(LED_BUILTIN, HIGH);
}
