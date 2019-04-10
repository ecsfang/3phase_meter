/***************************************************************************
  POWER METER - with 3 phases and blink detection
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include "3phase_adc.h"

#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <coredecls.h>                  // settimeofday_cb()
#include <Timezone.h>   // https://github.com/JChristensen/Timezone
#include <Ticker.h>

#include "EmonLib.h"


//### LOCAL SETTINGS ##########################################
#include "mySSID.h"         // Include private SSID and password etc ...
const char* otaHost     = "PowerMeterOTA";
const char* mqttClient  = "PowerMeterJN";

#define TOPIC        "powermeter2"
#define RX_DEBUG            // Some additional printouts ...
//#define USE_BLINK_INTERRUPT // Count blinks on the powermeter
#define USE_MQTT            // Remove if running in e.g. a test environment ...
#define NR_OF_PHASES  3     // Number of phases to watch
#define SCT_013_000         // The sensor used
#define USE_STATUS          // Define to send status message every X minute
#define STATUS_TIME   15    // Seconds between status messages ...
//#############################################################

#ifdef USE_BLINK_INTERRUPT
// The blinking LED on the meter could be connected here ...
const int blinkPin =  D0;
#endif
//const int sclPin =            D1;
//const int sdaPin =            D2;

#if defined(SCT_013_000)
// and for the YHDC SCT-013-000 CT sensor:
#define IP  100       // 100 A
#define IPC 0.05      // 50 mA
#define RT  (IP/IPC)  // Rt = 100 A ÷ 50 mA = 2000
#define RB  120        // Burden resistor
#define CORR_CURRENT  (RT/RB)
#elif defined(SCT_013_030)
// Ip is the rated primary current, and Vs is the output voltage at that current, then
// current constant = Ip ÷ Vs
// and for the YHDC SCT-013-000 CT sensor:
#define IP  30        // 30 A
#define VS  1   // 1V
#define CORR_CURRENT  (IP/VS)
#else
#error Must select which sensor to use!
#endif

// Create  instances for each CT channel
EnergyMonitor ct[NR_OF_PHASES];

// The pins connected to the sensors
int sctPin[NR_OF_PHASES] = { PHASE_CH1, PHASE_CH2, PHASE_CH3 };

WiFiClient espClient;
PubSubClient client(espClient);

// Check current with 1Hz ...
unsigned long delayTime = 1000;

// Set up summer/winter-time rules
TimeChangeRule CEST = {"", Last, Sun, Mar, 2, 120};     
TimeChangeRule CET = {"", Last, Sun, Oct, 3, 60}; 

Timezone CE(CEST, CET);
TimeChangeRule *tcr;

WiFiUDP ntpUDP;

Ticker flipper;
bool  bSendStatus = false;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

uint32_t seconds(void) { return millis()/1000; }

time_t  getNTPtime(void) {
  return CE.toLocal(timeClient.getEpochTime(), &tcr);
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println(F("PowerMeter!"));

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef USE_BLINK_INTERRUPT
  pinMode(blinkPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(blinkPin), onPulse, FALLING);
#endif

  Serial.println(F("Blink LEDs ..."));
  for(int i=0; i<16; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // On ...
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  // Off ...
    delay(100);
  }

//  Serial.println(F("Init Wire ..."));
//  Wire.begin(sdaPin, sclPin);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  initAdc();

  Serial.println();

  // Calibration factor = CT ratio / burden resistance
  // the current constant is the value of current you want to
  // read when 1 V is produced at the analogue input
  for (int i = 0; i < NR_OF_PHASES; i++) {
    // Replace the default pin reader with the customized ads pin reader
    ct[i].inputPinReader = adcPinReader;
    ct[i].current(sctPin[i], CORR_CURRENT);
  }

  Serial.println(F("Init OTA ..."));

  ArduinoOTA.setHostname(otaHost);
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

  setSyncProvider( getNTPtime );
  timeClient.begin();
  
#ifdef USE_MQTT
  client.setServer(mqtt_server, 1883);
#endif
#ifdef USE_STATUS
  flipper.attach(STATUS_TIME, doSendStatus);
#endif
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
#ifdef RX_DEBUG
    Serial.print("Attempting another MQTT connection...");
#endif
    // Attempt to connect
    if (client.connect(mqttClient)) {
#ifdef RX_DEBUG
      Serial.println("Connected!");
#endif
      // Once connected, publish an announcement...
      client.publish(MESSAGE, "ready");
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
time_t local = 0;

void loop()
{
  static int  prevHour = 0;

#ifdef USE_MQTT
  if (!client.connected())
    reconnect();

  if (client.connected())
    client.loop();
#endif

  ArduinoOTA.handle();

  if ( bBlink ) {
#ifdef RX_DEBUG
    Serial.print("Blinks: ");
    Serial.println(blinkCnt);
#endif
    bBlink = false;
  }

  // Get current time ...
  local = now();
  if( hour(local) != prevHour ) {
    // New hour ...
    if( hour(local) == 0 && prevHour == 24 ) {
      // Just passed midnight ... ;)
      runAtMidnight();
    }
    prevHour = hour(local);
  }

//  testADC();
  read3Phase();

  if( bSendStatus )
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

void sendMsg(const char *topic, const char *m)
{
  #define MSG_LEN 50
  char msg[MSG_LEN];

  snprintf (msg, MSG_LEN, "%s/%s", MESSAGE, topic);
#ifdef RX_DEBUG
  Serial.print("Publish message: ");
  snprintf (msg, MSG_LEN, "%s/%s", TOPIC, topic);
  Serial.print(msg);
  Serial.print(" ");
  Serial.println(m);
#endif
#ifdef USE_MQTT
  client.publish(msg, m);
#endif
}

void sendMsgF(const char *topic, double v)
{
  char buf[32];
  dtostrf(v, 1, 2, buf);
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
double        peakCurrent[NR_OF_PHASES];    // Peak current (per day)
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

void doSendStatus(void)
{
  bSendStatus = true;
}

void sendStatus(void)
{
  char  values[BUF_LEN];
  char  dateBuf[32];
  int   n = 0;

  // Param 0
  n = sprintf(values, "VALUES:%ld;", todayPower);
  // Param 1
  n += sprintf(values+n, "%ld;", yesterdayPower);

  for( int c=0; c<NR_OF_PHASES; c++ ) {
    // CURRENT
    dtostrf(irms[c],1,1,values+n);
    n = strlen(values);
    n += sprintf(values+n, ";");
    // POWER
    n += sprintf(values+n, "%d;", RMSPower[c]);
    // TODAY
    dtostrf(kilos[c],1,1,values+n);
    n = strlen(values);
    n += sprintf(values+n, ";");
    // PEAK
    dtostrf(peakCurrent[c],1,1,values+n);
    n = strlen(values);
    n += sprintf(values+n, ";");
  }

  // Let par point into the values string on the different values
  char *par[SENSOR_PAR_CNT];
  uint8_t cnt = 0;
  char *p = strstr(values, ":");
  while (p && cnt < SENSOR_PAR_CNT) {
    *(p++) = 0;
    par[cnt++] = p;
    p = strchr(p, ';');
  }
  
  // Fill in report
  String json;
  json = R"(
    {
      "Time": $TIME,
      "ENERGY": {
        "Total": $TOTAL,
        "Yesterday":$YDAY,
        "Phase1": {
          "Today": $TODAY_1,
          "Current": $CURRENT_1,
          "Power": $POWER_1,
          "Peak": $PEAK_1
        },
        "Phase2": {
          "Today": $TODAY_2,
          "Current": $CURRENT_2,
          "Power": $POWER_2,
          "Peak": $PEAK_2
        },
        "Phase3": {
          "Today": $TODAY_3,
          "Current": $CURRENT_3,
          "Power": $POWER_3,
          "Peak": $PEAK_3
        }
      }
    }
  )";

  local = now();
  sprintf(dateBuf, "%d.%02d.%02d %02d:%02d:%02d", year(local), month(local), day(local), hour(local), minute(local), second(local));

  json.replace("$TIME",       dateBuf);
  json.replace("$TOTAL",      par[0]);
  json.replace("$YDAY",       par[1]);
  json.replace("$CURRENT_1",  par[2]);
  json.replace("$POWER_1",    par[3]);
  json.replace("$TODAY_1",    par[4]);
  json.replace("$PEAK_1",     par[5]);
  json.replace("$CURRENT_2",  par[6]);
  json.replace("$POWER_2",    par[7]);
  json.replace("$TODAY_2",    par[8]);
  json.replace("$PEAK_2",     par[9]);
  json.replace("$CURRENT_3",  par[10]);
  json.replace("$POWER_3",    par[11]);
  json.replace("$TODAY_3",    par[12]);
  json.replace("$PEAK_3",     par[13]);

#ifdef RX_DEBUG
  Serial.println( json );
#endif
  sendMsg("status", json.c_str());
  bSendStatus = false;
}


void read3Phase(void)
{
  uint8_t upd = 0;
  char topic[16];
#ifdef RX_DEBUG
  int n = 0;
  char buffer[128];
#endif

  digitalWrite(LED_BUILTIN, LOW);

  for ( int c = 0; c < NR_OF_PHASES; c++) {
    // Read the current value of phase 'c'
    irms[c] = ct[c].calcIrms(1480);
    RMSPower[c] = 230*irms[c];

#ifdef RX_DEBUG
    dtostrf(irms[c],1,2,buffer+n);
    n = strlen(buffer);
    n += sprintf(buffer+n, "  ");
#endif

    if( change(irms[c], oldIrms[c], DELTA_AMP) ) {
      // Value has changed - mark for update!
      upd |= 1<<c;
      oldIrms[c] = irms[c];
    }

    // A new peak-value ... ?
    if( irms[c] > peakCurrent[c] )
      peakCurrent[c] = irms[c];
    if( RMSPower[c] > peakPower[c] )
      peakPower[c] = RMSPower[c];

    // Get time since last reading ...
    endMillis[c] = millis();
    unsigned long dTime = (endMillis[c] - startMillis[c]);
    startMillis[c] = endMillis[c];
    // How much power has been used ... ?
    double duration = ((double)dTime)/(60*60*1000.0); // Time in hours since last reading
    kilos[c] += RMSPower[c] * duration / 1000; // So many kWh have been used ...
  }

#ifdef RX_DEBUG
  Serial.println(buffer);
#endif

  if( upd ) { // Have updates to send ...
    for ( int c = 0; c < NR_OF_PHASES; c++) {
      if( upd & (1<<c) ) {
        sprintf(topic, "phase_%d", c+1);
        sendMsgF(topic, irms[c]);
      }
    }
  }

  digitalWrite(LED_BUILTIN, HIGH);
}
