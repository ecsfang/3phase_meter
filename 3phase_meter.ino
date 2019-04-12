/***************************************************************************
  POWER METER - with 3 phases and blink detection
 ***************************************************************************/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <Wire.h>
#include <SPI.h>
#include "3phase_adc.h"

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic


#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <ArduinoOTA.h>
#include <PubSubClient.h>

//#define FIRST_FLASH

// For the 0.96" OLED display
#include <Adafruit_GFX.h>attempt
#include <Adafruit_SSD1306.h>

// For timekeeping with correct daylight savings ...
#include <NTPClient.h>
#include <time.h>                 // time() ctime()
#include <sys/time.h>             // struct timeval
#include <coredecls.h>            // settimeofday_cb()
#include <Timezone.h>             // https://github.com/JChristensen/Timezone

// For doing repetitive jobs
#include <Ticker.h>

// Library for current and power calculations
#include "EmonLib.h"


//### LOCAL SETTINGS ##########################################
#include "mySSID.h"               // Include private SSID and password etc ...
const char* otaHost     = "PowerMeterOTA";
const char* mqttClient  = "PowerMeterJN";

#define MESSAGE           "powermeter2" // Default message
#define RX_DEBUG                  // Some additional printouts ...
//#define USE_BLINK_INTERRUPT       // Count blinks on the powermeter
#define USE_MQTT                  // Remove if running in e.g. a test environment ...
#define NR_OF_PHASES  3           // Number of phases to watch
#define SCT_013_000               // The sensor used
#define USE_STATUS                // Define to send status message every X minute
#define STATUS_TIME   15          // Seconds between status messages ...
//#############################################################

#ifdef USE_BLINK_INTERRUPT
// The blinking LED on the energy meter could be connected here ...
const int blinkPin =  D0;
#endif

// The pins connected to the I2C connector (display)
//const int sclPin =            D1;
//const int sdaPin =            D2;

#if defined(SCT_013_000)
  // Settings for the YHDC SCT-013-000 CT sensor
  // Measure current (50mA equal to 100A through the sensor)
  #define IP  100       // 100 A
  #define IPC 0.05      // 50 mA
  #define RT  (IP/IPC)  // Rt = 100 A ÷ 50 mA = 2000
  #define RB  120        // Burden resistor
  #define CORR_CURRENT  (RT/RB)
#elif defined(SCT_013_030)
  // Settings for the YHDC SCT-013-030 CT sensor
  // Measure voltage (1V equal to 30A through the sensor)
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
int sctPin[NR_OF_PHASES] = { ADC_CH0, ADC_CH1, ADC_CH2 };

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

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define OLED_RESET   0 // Reset pin # (or -1 if sharing Arduino reset pin)
//Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 OLED(OLED_RESET);

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

uint32_t seconds(void) { return millis()/1000; }

time_t  getNTPtime(void) {
  return CE.toLocal(timeClient.getEpochTime(), &tcr);
}

// Flag for saving data
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

char mqtt_server[40];
char mqtt_port[6] = "8080";
char mqtt_user[16];
char mqtt_pass[16];
char mqtt_msg[32];

void oled_setup()   {
  OLED.begin();
  OLED.clearDisplay();

  //Add stuff into the 'display buffer'
  OLED.setTextColor(WHITE);
  OLED.setTextSize(2);
  OLED.setCursor(0,0);
  OLED.println("PowerMeter");
//  OLED.setTextSize(0);
//  OLED.println("Check AP!");

  OLED.display(); //output 'display buffer' to screen  
//  OLED.startscrollleft(0x00, 0x0F); //make display scroll 
}

void setup() {

  oled_setup();

  Serial.begin(115200);
  delay(3000);
  Serial.println(F("PowerMeter!"));

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef USE_BLINK_INTERRUPT
  pinMode(blinkPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(blinkPin), onPulse, FALLING);
#endif

  Serial.println(F("Blink LEDs ..."));
  for(int i=0; i<4; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // On ...
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  // Off ...
    delay(100);
  }

  sprintf( mqtt_msg, "%s", MESSAGE);

#ifdef FIRST_FLASH
  //clean FS for testing 
  SPIFFS.format();
#endif

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(256);
        DeserializationError error = deserializeJson(doc, buf.get());
        if (error) {
        }
        JsonObject json = doc.as<JsonObject>();
        serializeJson(json, Serial);
        if (!json.isNull()) {
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_msg,  json["mqtt_msg"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 20);
  WiFiManagerParameter custom_mqtt_msg("msg",   "mqtt msg",  mqtt_msg, 30);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

#ifdef FIRST_FLASH
  // Reset Wifi settings for testing  
  wifiManager.resetSettings();
#endif

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
//  wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_msg);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration

  OLED.clearDisplay();
  //Add stuff into the 'display buffer'
  OLED.setTextColor(WHITE);
  OLED.setTextSize(1);
  OLED.setCursor(0,10);
  OLED.println("Check AP!");

  OLED.display(); //output 'display buffer' to screen  
  OLED.startscrollleft(0x00, 0x0F); //make display scroll 

  
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  OLED.clearDisplay();
  OLED.stopscroll();
  //Add stuff into the 'display buffer'
  OLED.setTextColor(WHITE);
  OLED.setTextSize(1);
  OLED.setCursor(0,10);
  OLED.println("Connected!");

  OLED.display(); //output 'display buffer' to screen  

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_msg,  custom_mqtt_msg.getValue());

  Serial.println("MQTT setttings:");
  Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  Serial.println(mqtt_user);
  Serial.println(mqtt_pass);
  Serial.println(mqtt_msg);
  Serial.println();

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument doc(256);
    JsonObject json = doc.to<JsonObject>();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_msg"] = mqtt_msg;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
    //end save
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

//  setSyncProvider( getNTPtime );
  timeClient.begin();
  
#ifdef USE_STATUS
  flipper.attach(STATUS_TIME, doSendStatus);
#endif

#ifdef USE_MQTT
  const uint16_t mqtt_port_x = atoi(mqtt_port); 
  client.setServer(mqtt_server, mqtt_port_x);
//  client.setServer(mqtt_server, 1883);
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
      client.publish(mqtt_msg, "ready");
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

  if( (tCnt % 10) == 0 )
    timeClient.update();

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
  local = getNTPtime();
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

  snprintf (msg, MSG_LEN, "%s/%s", mqtt_msg, topic);
#ifdef RX_DEBUG
  Serial.print("Publish message: ");
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

#define DELTA_AMP 0.5
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
  todayPower = 0;
  yesterdayPower = 0;
  for ( int c = 0; c < NR_OF_PHASES; c++) {
    yesterdayPower += (unsigned long)kilos[c];
    kilos[c] = 0.0;
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

  local = getNTPtime(); //timeClient.getEpochTime(); //now();
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
//  Serial.println(timeClient.getFormattedTime());
//  Serial.println( json );
#endif
//  Serial.println(timeClient.getFormattedTime());
  sendMsg("status", json.c_str());
  bSendStatus = false;
}


void read3Phase(void)
{
  uint8_t upd = 0;
  char topic[16];
  double  kilosNow = 0;
#ifdef RX_DEBUG
  int n = 0;
  int n1 = 0;
  char buffer[128];
#endif

  digitalWrite(LED_BUILTIN, LOW);

  OLED.begin();
  OLED.clearDisplay();
  OLED.setTextColor(WHITE);
  OLED.setTextSize(0);
  OLED.setCursor(0,0);

  for ( int c = 0; c < NR_OF_PHASES; c++) {
    // Read the current value of phase 'c'
    irms[c] = ct[c].calcIrms(1480);
    RMSPower[c] = 230*irms[c];

    //Add stuff into the 'display buffer'
    n1 = sprintf(topic, "F%d: ", c+1);
    dtostrf(irms[c],1,2,topic+n1);
    n1 = strlen(topic);
    sprintf(topic+n1, "A");
    OLED.println(topic);

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
    kilosNow = RMSPower[c] * duration; // So many Wh have been used ...
    kilos[c] += kilosNow/1000;
    todayPower += kilosNow;
  }

  //Add stuff into the 'display buffer'
  OLED.setTextSize(1);
  dtostrf(todayPower,1,1,topic);
  n1 = strlen(topic);
  sprintf(topic+n1, "Wh");
  OLED.println(topic);

  OLED.display(); //output 'display buffer' to screen  

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
