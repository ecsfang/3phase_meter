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

#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <coredecls.h>                  // settimeofday_cb()
#include <Timezone.h>   // https://github.com/JChristensen/Timezone

#include "EmonLib.h"


//### LOCAL SETTINGS ##########################################
#include "mySSID.h"         // Include private SSID and password etc ...
const char* otaHost     = "PowerMeterOTA";
const char* mqttClient  = "PowerMeterJN";

#define RX_DEBUG            // Some additional printouts ...
#define USE_BLINK_INTERRUPT // Count blinks on the powermeter
#define USE_MQTT            // Remove if running in e.g. a test environment ...
#define NR_OF_PHASES  3     // Number of phases to watch
#define SCT_013_000         // The sensor used
//#############################################################

#ifdef USE_BLINK_INTERRUPT
// The blinking LED on the meter could be connected here ...
const int blinkPin =          D0;
#endif
const int sclPin =            D1;
const int sdaPin =            D2;

#if defined(SCT_013_000)
// and for the YHDC SCT-013-000 CT sensor:
#define IP  100       // 100 A
#define IPC 0.05      // 50 mA
#define RT  (IP/IPC)  // Rt = 100 A ÷ 50 mA = 2000
#define RB  78        // Burden resistor
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

bool running = false;
long statusStart = 0L;
// Seconds between status messages ...
#define STATUS_TIME (1000*15) // (1000*60*5) // 5 minutes ...

// Create  instances for each CT channel
EnergyMonitor ct[NR_OF_PHASES];

// The pins connected to the sensors
int sctPin[NR_OF_PHASES] = { ADC_CH0, ADC_CH1, ADC_CH2 };

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_LEN 50
char msg[MSG_LEN];

unsigned long delayTime = 1000;

#define TZ              1       // (utc+) TZ in hours
#define DST_MN          0 //60      // use 60mn for summer time in some countries

////////////////////////////////////////////////////////

#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)

timeval cbtime;      // time set in callback
bool cbtime_set = false;


void time_is_set(void) {
  gettimeofday(&cbtime, NULL);
  cbtime_set = true;
  Serial.println("------------------ settimeofday() was called ------------------");
}

// for testing purpose:
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

TimeChangeRule CEST = {"", Last, Sun, Mar, 2, 120};     
TimeChangeRule CET = {"", Last, Sun, Oct, 3, 60}; 
Timezone CE(CEST, CET);
TimeChangeRule *tcr;

#define PTM(w) \
  Serial.print(":" #w "="); \
  Serial.print(tm->tm_##w);

void printTm(const char* what, const tm* tm) {
  Serial.print(what);
  PTM(isdst); PTM(yday); PTM(wday);
  PTM(year);  PTM(mon);  PTM(mday);
  PTM(hour);  PTM(min);  PTM(sec);
}

timeval tv;
timespec tp;
time_t now;
uint32_t now_ms, now_us;

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println(F("PowerMeter!"));

  settimeofday_cb(time_is_set);

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

  Serial.println(F("Init Wire ..."));
  Wire.begin(sdaPin, sclPin);

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
    ct[i].inputPinReader = adcPinReader; // Replace the default pin reader with the customized ads pin reader
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

  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");
  // don't wait, observe time changing when ntp timestamp is received
  
#ifdef USE_MQTT
  client.setServer(mqtt_server, 1883);
#endif
  running = true;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
#ifdef RX_DEBUG
    Serial.print("Attempting another MQTT connection...");
#endif
    // Attempt to connect
    if (client.connect(mqttClient)) {
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

  gettimeofday(&tv, nullptr);
  clock_gettime(0, &tp);
  now = time(nullptr);
  time_t cet=CE.toLocal(now(),&tcr);
  setTime(cet);
  now_ms = millis();
  now_us = micros();
/**
  // localtime / gmtime every second change
  static time_t lastv = 0;
  if (lastv != tv.tv_sec) {
    lastv = tv.tv_sec;
    Serial.println();
    printTm("localtime", localtime(&now));
    Serial.println();
    printTm("gmtime   ", gmtime(&now));
    Serial.println();
    Serial.println();
  }

  // time from boot
  Serial.print("clock:");
  Serial.print((uint32_t)tp.tv_sec);
  Serial.print("/");
  Serial.print((uint32_t)tp.tv_nsec);
  Serial.print("ns");

  // time from boot0
  Serial.print(" millis:");
  Serial.print(now_ms);
  Serial.print(" micros:");
  Serial.print(now_us);

  // EPOCH+tz+dst
  Serial.print(" gtod:");
  Serial.print((uint32_t)tv.tv_sec);
  Serial.print("/");
  Serial.print((uint32_t)tv.tv_usec);
  Serial.print("us");

  // EPOCH+tz+dst
  Serial.print(" time:");
  Serial.print((uint32_t)now);

  // human readable
  Serial.print(" ctime:(UTC+");
  Serial.print((uint32_t)(TZ * 60 + DST_MN));
  Serial.print("mn)");
  Serial.print(ctime(&now));
***************************************/

  if ( bBlink ) {
    Serial.print("Blinks: ");
    Serial.println(blinkCnt);
    bBlink = false;
  }

//  testADC();
  read3Phase();

  if (running && ((millis() - statusStart) >= STATUS_TIME)) {
    statusStart += STATUS_TIME; // this prevents drift in the delays

    Serial.println(F("Send status ..."));
    sendStatus();
  }
  
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
unsigned long peakCurrent[NR_OF_PHASES];    // Peak current (per day)
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
  for( int c=0; c<NR_OF_PHASES; c++ ) {
    dtostrf(irms[0],1,1,values+n);
    n = strlen(values);
    n += sprintf(values+n, ";");
  }
  for( int c=0; c<NR_OF_PHASES; c++ )
    n += sprintf(values+n, "%d;", RMSPower[c]);
  for( int c=0; c<NR_OF_PHASES; c++ ) {
    dtostrf(kilos[0],1,1,values+n);
    n = strlen(values);
    n += sprintf(values+n, ";");
  }
  for( int c=0; c<NR_OF_PHASES; c++ ) {
    dtostrf(peakPower[0],1,1,values+n);
    n = strlen(values);
    n += sprintf(values+n, ";");
  }

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

  time_t local = CE.toLocal(utc, &tcr);  
  sprintf(msg, "%02d:%02d", hour(local), minute(local)); 

  tm *ptm = localtime(&now);
  sprintf(msg, "%d-%02d-%02d", ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday);
  
  json.replace("$TIME",       msg);
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
  uint8_t upd = 0;
  char topic[16];
  char buffer[128];
  int nn, n = 0;

  digitalWrite(LED_BUILTIN, LOW);

  for ( int c = 0; c < NR_OF_PHASES; c++) {
    irms[c] = ct[c].calcIrms(1480);
    RMSPower[c] = 230*irms[c];

    dtostrf(irms[c],1,2,buffer+n);
    n = strlen(buffer);
    n += sprintf(buffer+n, "  ");

    if( change(irms[c], oldIrms[c], DELTA_AMP) ) {
      upd |= 1<<c;
      oldIrms[c] = irms[c];
    }
    if( irms[c] > peakCurrent[c] )
      peakCurrent[c] = irms[c];
    if( RMSPower[c] > peakPower[c] )
      peakPower[c] = RMSPower[c];
    // Get time since last reading ...
    endMillis[c] = millis();
    unsigned long time = (endMillis[c] - startMillis[c]);
    startMillis[c] = endMillis[c];
    // How much power has been used ... ?
    double duration = ((double)time)/(60*60*1000.0); // Time in hours since last reading
    kilos[c] += RMSPower[c] * duration / 1000; // So many kWh have been used ...
  }

  Serial.println(buffer);
  
  for ( int c = 0; c < NR_OF_PHASES; c++) {
    if( upd & (1<<c) ) {
      sprintf(topic, "phase_%d", c+1);
      sendMsgF(topic, irms[c]);
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
}
