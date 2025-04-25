/***************************************************************************
  POWER METER - with 3 phases and blink detection

  Arduino settings:
  Card:       Lolin (Wemos) D1 R2 & mini
  Flash Size: 4M (1M Spiffs)

  Libraries:
    ADAFruit_SSD1306  2.5.9
    NTPClient         3.2.1
    Timezone          1.2.4
    MCP3XXX           1.0.0
    WiFiManager       2.0.17
    PubSubClient      2.8.0
    EmonLib           Use updated lib in repositoryt
    ArduinoJson       6.21.2
    RemoteDebug       3.0.5

  Board:
    ESP8266           2.5.1

 TimeZone warning:
    open the library.properties file with a text editor
    change the architectures=avr line to "architectures=avr,esp8266, esp32" (or "architectures=*")
    save and close the file
 ***************************************************************************/
#include <FS.h> //this needs to be first, or it all crashes and burns...

#include <Wire.h>
#include <SPI.h>
#include "3phase_utils.h"
#include "3phase_adc.h"

#if 0
#  include <WiFi.h>
#  include <DNSServer.h>
#  include <ESP8266WebServer.h>
#  include <WiFiManager.h>
#else
#  include <ESP8266WiFi.h>      //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#  include <DNSServer.h>        //Local DNS Server used for redirecting all requests to the configuration portal
#  include <ESP8266WebServer.h> //Local WebServer used to serve the configuration portal
#  include <WiFiManager.h>      //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#endif

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include <ArduinoOTA.h>
#include <PubSubClient.h>

//#include "config.h"
//configData *cfgdat;

#ifdef USE_REMOTE_DBG
#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug
#endif

// For doing repetitive jobs
#include <Ticker.h>

// Library for current and power calculations
#include "EmonLib.h"

//### LOCAL SETTINGS ##########################################
#include "mySSID.h" // Include private SSID and password etc ...
const char *otaHost =    METER "MeterOTA";
const char *mqttClient = METER "MeterTF";

//#define RX_DEBUG              // Some additional printouts ...
//#define USE_BLINK_INTERRUPT       // Count blinks on the powermeter
#define USE_STATUS          // Define to send status message every X minute
#ifdef RX_DEBUG
#define STATUS_TIME 20      // Seconds between status messages ...
#else
#define STATUS_TIME 5 * 60  // Seconds between status messages ...
#endif
#define USE_MQTT           // Remove if running in e.g. a test environment ...
//#define FIRST_FLASH
//#define USE_TEST_DATA
#define RESET_AT_MIDNIGHT
#define MSG_LEN 1024
#define LWD_TIMEOUT  15*1000  // Reboot if loop watchdog timer reaches this time out value
#define DISPLAY_TIME  10000   // How long the display should be active after boot
//#############################################################

#ifdef USE_BLINK_INTERRUPT
// The blinking LED on the energy meter could be connected here ...
const int blinkPin = D0;
#endif

#if defined(SCT_013_000)

// Settings for the YHDC SCT-013-000 CT sensor
// Measure current (50mA equal to 100A through the sensor)
#define IP 100        // 100 A
#define IPC 0.05      // 50 mA
#define RT (IP / IPC) // Rt = 100 A ÷ 50 mA = 2000
#define CORR_CURRENT (RT / RB)

#elif defined(SCT_013_030)

// Settings for the YHDC SCT-013-030 CT sensor
// Measure voltage (1V equal to 30A through the sensor)
// Ip is the rated primary current, and Vs is the output voltage at that current, then
// current constant = Ip ÷ Vs
// and for the YHDC SCT-013-000 CT sensor:
#define IP 30 // 30 A
#define VS 1  // 1V
#define CORR_CURRENT (IP / VS)

#else
#error Must select which sensor to use!
#endif

// Create  instances for each CT channel
EnergyMonitor ct[NR_OF_PHASES];

// The pins connected to the sensors
int sctPin[NR_OF_PHASES] = {
  ADC_CH0, ADC_CH1, ADC_CH2
#if NR_OF_PHASES == 4
  , ADC_CH3
#endif
};

WiFiClient espClient;
PubSubClient client(espClient);
#ifdef USE_REMOTE_DBG
RemoteDebug Debug;
#endif

// Housekeeping ....
unsigned long startMillis[NR_OF_PHASES];
unsigned long endMillis[NR_OF_PHASES];
double oldIrms[4] = {-99, -99, -99, -99};
int oldPower[4] = {-99, -99, -99, -99};

// Current values ...
double irms[NR_OF_PHASES];
unsigned long RMSPower[NR_OF_PHASES];  // Current power (W)
double peakCurrent[NR_OF_PHASES];      // Peak current (per day)
unsigned long peakPower[NR_OF_PHASES]; // Peak power (per day)
double kilos[NR_OF_PHASES];            // Total kWh today (per phase)
double todayPower;              // Todays total
double currentPower;           // Current energy in kWh
double yesterdayPower;          // Yesterdays total

// Set up summer/winter-time rules
TimeChangeRule CEST = {"", Last, Sun, Mar, 2, 120};
TimeChangeRule CET = {"", Last, Sun, Oct, 3, 60};

Timezone CE(CEST, CET);
TimeChangeRule *tcr;


Ticker flipper;
bool bSendStatus = false;

#ifdef USE_DISPLAY
extern Adafruit_SSD1306 OLED;
#define DRAW_ROTATING_DISC
#endif

Ticker lwdTicker;

unsigned long lwdTime = 0;
unsigned long lwdTimeout = LWD_TIMEOUT;

// Define an NTP client to get date and time.
WiFiUDP ntpUDP;
// By default 'pool.ntp.org' is used with 60 seconds update interval and no offset
NTPClient timeClient(ntpUDP);

uint32_t seconds(void) { return millis() / 1000; }

time_t getNTPtime(void)
{
  return CE.toLocal(timeClient.getEpochTime(), &tcr);
}

void ICACHE_RAM_ATTR lwdtcb(void) 
{
  if ((millis() - lwdTime > LWD_TIMEOUT) || (lwdTimeout - lwdTime != LWD_TIMEOUT))
  {
    ESP.restart();  
  }
}

void lwdtFeed(void) {
  lwdTime = millis();
  lwdTimeout = lwdTime + LWD_TIMEOUT;
}

// Flag for saving data
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

char mqtt_server[40];
char mqtt_port[6] = "1883";
char mqtt_user[32];
char mqtt_pass[32];
char mqtt_msg[32];

void setup()
{

  Serial.begin(115200);

  oled_setup();

  delay(1000);
  Serial.println(F("PowerMeter!"));

  pinMode(LED_BUILTIN, OUTPUT);

#ifdef USE_BLINK_INTERRUPT
  pinMode(blinkPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(blinkPin), onPulse, FALLING);
#endif

#ifdef USE_DISPLAY
  ClrDisplay();
  //Add stuff into the 'display buffer'
  DispText(0, 10, "Booting");
  UpdateDisplay(true);
#endif

  Serial.println(F("Blink LEDs ..."));
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, LOW); // On ...
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); // Off ...
    delay(100);
  }

  sprintf(mqtt_msg, "%s", MESSAGE);

#ifdef FIRST_FLASH
  //clean FS for testing
  Serial.println("Format FS...");
  SPIFFS.format();
#endif

#if 0
    // instantiate the `configData` object, during construction
    // it will read the configuration file and make its data 
    // availble.
    cfgdat = new configData();

    // let's check for errors before continuing...
    String errMsg;
    if(!cfgdat->getError(errMsg)) {
        // no errors!
        
        Serial.println("Mounting FS...");
        // set up the logger, typically sent out via Serial
        // but can be modified as needed.

        WiFi.mode(WIFI_STA);
        // Use the functions that return `const char *` because
        // WiFi.begin() doesn't take String as an arg.
        WiFi.begin(cfgdat->getSSID(), cfgdat->getPASS());
    } else {
        // an error has occurred, the message will provide
        // additional information.
        Serial.println(errMsg);
    }
#endif

  Serial.println("WiFi setttings:");
  Serial.print("SSID: ");
  Serial.print(ssid);
  Serial.print(":");
  Serial.println(password);

    WiFi.mode(WIFI_STA);
    // Use the functions that return `const char *` because
    // WiFi.begin() doesn't take String as an arg.
    WiFi.begin(ssid, password);

    int nErr = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        nErr++;
        if( nErr > 30 ) {
          Serial.println("\nRestarting!");
          delay(500);
          ESP.restart();
        }
    }
    
#ifdef USE_DISPLAY
  ClrDisplay();
  //Add stuff into the 'display buffer'
  DispText(0, 10, "Connected!");
  DispText(0, 20, ip());
  UpdateDisplay(false);
  delay(2000);
  ClrDisplay();
#endif

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, _mqtt_server); //cfgdat->getMQServer() ); //custom_mqtt_server.getValue());
  strcpy(mqtt_port, _mqtt_port); //cfgdat->getMQPort() ); //custom_mqtt_port.getValue());
  strcpy(mqtt_user, _mqtt_user); //cfgdat->getMQUser() ); //custom_mqtt_user.getValue());
  strcpy(mqtt_pass, _mqtt_pass); //cfgdat->getMQPass() ); //custom_mqtt_pass.getValue());

#if 1
  Serial.println("MQTT setttings:");
  Serial.print("Server: ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);
  Serial.print("User:   ");
  Serial.println(mqtt_user);
  Serial.print("Passw:  ");
  Serial.println(mqtt_pass);
  Serial.print("Msg:    ");
  Serial.println(mqtt_msg);
  Serial.println();
#endif
#if 0
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
#endif

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

#ifdef USE_MQTT
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
    switch( error ) {
    case OTA_AUTH_ERROR:    Serial.println("Auth Failed");     break;
    case OTA_BEGIN_ERROR:   Serial.println("Begin Failed");    break;
    case OTA_CONNECT_ERROR: Serial.println("Connect Failed");  break;
    case OTA_RECEIVE_ERROR: Serial.println("Receive Failed");  break;
    case OTA_END_ERROR:     Serial.println("End Failed");      break;
    default:                Serial.println("Unknown error!");
    }
  });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
#endif

  // Initialize the NTPClient
  timeClient.begin();

#ifdef USE_STATUS
  flipper.attach(STATUS_TIME, doSendStatus);
#endif

#ifdef USE_MQTT
  const uint16_t mqtt_port_x = atoi(mqtt_port);
  client.setBufferSize(1024);
  client.setServer(mqtt_server, mqtt_port_x);
#endif

  client.setCallback(callback);

  client.publish(mqtt_msg, "init");
  client.subscribe("powerMeter/disp");

#ifdef USE_REMOTE_DBG
  // Initialize the server (telnet or web socket) of RemoteDebug
  Debug.begin( MESSAGE );
  delay(500);
  Debug.printf("Hello from " MESSAGE "!\n");

  Debug.printf("WiFi connected\nIP address: ");
  Debug.println(WiFi.localIP());
  Debug.print("ESP Board MAC Address:  ");
  Debug.println(WiFi.macAddress());
#endif

  lwdtFeed();
  lwdTicker.attach_ms(LWD_TIMEOUT, lwdtcb); // attach lwdt callback routine to Ticker object
 
//  ESP.wdtDisable();
}

char *ip(void)
{
  static char ipb[32];
  sprintf(ipb, "\"%s\"", WiFi.localIP().toString().c_str());
  return ipb;
}

int m_display = DISPLAY_TIME;

void callback(char* topic, byte* payload, unsigned int length) {
  //Message arrived [powerMeter/disp]
  if( !strcmp(topic, "powerMeter/disp" ) ) {
    Serial.print("Init display ...");
    m_display = DISPLAY_TIME;
  }
  if( !strcmp(topic, "powerMeter/reset" ) ) {
    Serial.print("Reset device ...");
    delay(500);
    ESP.restart();
  }
  if( !strcmp(topic, "powerMeter/clear" ) ) {
    Serial.print("Clear device ...");
    todayPower = 0;
    runAtMidnight();
    m_display = DISPLAY_TIME;
    ClrDisplay();
  }
}

#ifdef USE_DISPLAY
void dispError(char *error)
{
  OLED.clearDisplay();
  //Add stuff into the 'display buffer'
  OLED.setTextColor(WHITE);
  OLED.setTextSize(1);
  OLED.setCursor(0, 10);
  OLED.print(error);
  OLED.display();                   //output 'display buffer' to screen
//  OLED.startscrollleft(0x00, 0x0F); //make display scroll
}
#endif

void reconnect()
{
  int nLoop = 0;
  // Loop until we're reconnected
  while (!client.connected()) {
#ifdef RX_DEBUG
    Serial.println("Attempting another MQTT connection ...");
    Serial.print("* ");
    Serial.println(mqttClient);
    Serial.print("* ");
    Serial.println(mqtt_user);
    Serial.print("* ");
    Serial.println(mqtt_pass);
    Serial.println("---------------------------------------");
#endif
    // Attempt to connect
    if (client.connect(mqttClient, mqtt_user, mqtt_pass)) {
#ifdef RX_DEBUG
      Serial.println("Connected!");
#endif
      // Once connected, publish an announcement...
      client.publish(mqtt_msg, "connect");
      client.subscribe("powerMeter/disp");
      ClrDisplay();
} else {
#ifdef USE_DISPLAY
      if( nLoop == 1 ) {
        char err[16];
        sprintf(err, "Err: %d", client.state());
        dispError(err);
        OLED.setCursor(0, 20);
        OLED.println(mqttClient);
        OLED.setCursor(0, 30);
        OLED.println(mqtt_user);
        OLED.setCursor(0, 50);
        OLED.println(mqtt_pass);
      }
#endif
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      if( nLoop++ > 5 ) {
        Serial.println("Restarting!");
#ifdef USE_DISPLAY
        dispError("Restarting!");
#endif
        delay(500);
        ESP.restart();
      }
    }
  }
  ClrDisplay();
#ifdef USE_REMOTE_DBG
  Debug.printf("Connected!\n");
#endif
}

bool bBlink = false;
int blinkCnt = 0;
unsigned int tCnt = 0;
time_t local = 0;
int pix = 0;

void loop()
{
  static int prevHour = 0;

  digitalWrite(LED_BUILTIN, LOW);

#ifdef USE_REMOTE_DBG
  // Remote debug over WiFi
  Debug.handle();
#endif

  ArduinoOTA.handle();

#ifdef USE_MQTT
  if (!client.connected())
    reconnect();
  client.loop();
#endif

  lwdtFeed();

  digitalWrite(LED_BUILTIN, HIGH);

#ifdef USE_DISPLAY
  if (m_display && (tCnt % 10) == 0) {
#ifdef DRAW_ROTATING_DISC
    // Draw moving line at bottom
    OLED.writePixel(pix, 63, WHITE);
    pix++;
    pix = pix % SCREEN_WIDTH;
    OLED.writePixel((pix + SCREEN_WIDTH - 10) % SCREEN_WIDTH, 63, BLACK);
    UpdateDisplay(false);
#endif
    m_display--;
    if( m_display == 0 ) {
       ClrDisplay();
       UpdateDisplay(false);
    }
  }
#endif

  // Every now and then ...
  if ((tCnt % 1000) == 0) {
#ifdef USE_REMOTE_DBG
    Debug.println("Check time ...");
#endif
    if ((tCnt % 100000) == 0)
      timeClient.update();

    // Get current time ...
    local = getNTPtime();
    if (hour(local) != prevHour) {
      // New hour ...
#ifdef RESET_AT_MIDNIGHT
      if (hour(local) == 0 && prevHour == 23)
#endif // else every hour!
      {
        // Just passed midnight ... ;)
        runAtMidnight();
      }
      prevHour = hour(local);
    }
  }

  if ((tCnt % 100) == 0) {

    if (bBlink) {
#ifdef RX_DEBUG
      Serial.print("Blinks: ");
      Serial.println(blinkCnt);
#endif
      bBlink = false;
    }

#ifdef USE_DISPLAY
    if( m_display ) {
#ifdef DRAW_ROTATING_DISC
      OLED.fillRect(0, 0, Wm, SCREEN_HEIGHT - 1, BLACK);
#else
      OLED.fillRect(0, 0, Wm, SCREEN_HEIGHT, BLACK);
#endif
    }
#endif

    //  testADC();
    read3Phase();

    if (bSendStatus) {
#ifdef USE_REMOTE_DBG
      Debug.println("Send status ...");
#endif
      sendStatus();
    }
  }
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
  char msg[MSG_LEN];

  snprintf(msg, MSG_LEN, "%s/%s", mqtt_msg, topic);
#ifdef USE_REMOTE_DBG
  Debug.print("Publish message (");
  Debug.print(strlen(m));
  Debug.print("): ");
  Debug.print(msg);
  Debug.print(" ");
  Debug.println(m);
#endif
#ifdef RX_DEBUG
  Serial.print("Publish message (");
  Serial.print(strlen(m));
  Serial.print("): ");
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
  snprintf(buf, 32, "%d", v);
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


double getCurrentPower(void)
{
  double e = currentPower;
  currentPower = 0.0;
  return e;
}
unsigned long getTodayPower(void)
{
  return (unsigned long)todayPower;
}

unsigned long getYesterdayPower(void)
{
  return (unsigned long)yesterdayPower;
}

void runAtMidnight(void)
{
  yesterdayPower = todayPower;
  todayPower = 0.0;
  for (int c = 0; c < NR_OF_PHASES; c++) {
    kilos[c] = 0.0;
    peakPower[c] = 0.0;
    peakCurrent[c] = 0.0;
  }
  doSendStatus();
}

void doSendStatus(void)
{
  bSendStatus = true;
}

int _rcnt = 0;

void read3Phase(void)
{
  uint8_t upd = 0;
  char topic[16];
  double wattNow = 0;
#ifdef RX_DEBUG
  int n = 0;
  char buffer[128];
#endif
  int iCurr;
  static int iUpd = 0;
  static unsigned int nReadings = 0;

  iUpd++;

#ifdef USE_CORRECTION
  double  corr[NR_OF_PHASES] = {
#ifdef KITCHEN
  0.24, 0.28, 0.05, 0.33
#else
0.13, 0.13, 0.13
//    2.4+0.2, 1.7+0.5, 2.2+0.2
#endif
  };
#endif

  currentPower = 0.0;

  // For each phase ...
  for (int c = 0; c < NR_OF_PHASES; c++) {
    // Read the current value of phase 'c'
#ifdef USE_TEST_DATA
    irms[c] = rand() % 30;
#else
#ifdef USE_CORRECTION
    irms[c] = ct[c].calcIrms(1480) - corr[c];
#else
    irms[c] = ct[c].calcIrms(1480);
#endif
if( irms[c] < 0.0 )
      irms[c] = 0.0;
#endif

    if( _rcnt < NR_OF_PHASES*5 ) { // Skip first readings to let sensor settle ...
      _rcnt++;
#ifdef RX_DEBUG
      n += sprintf(buffer + n, "<skipped>");
#endif
      continue;
    }
  
    RMSPower[c] = 230 * irms[c];

#ifdef USE_DISPLAY
    // Draw a bar with value on the display corresponding to the current
    if( m_display )
      DrawBar(c, irms[c]);
#endif

#ifdef RX_DEBUG
    // Add current to log buffer ...
    dtostrf(irms[c], 1, 2, buffer + n);
    n = strlen(buffer);
    n += sprintf(buffer + n, "  ");
#endif

    // If current has changed significantly, mark it for update!
    if (change(irms[c], oldIrms[c], DELTA_AMP)) {
      upd |= 1 << c;
      oldIrms[c] = irms[c];
    }

    // A new peak-value ... ?
    if (irms[c] > peakCurrent[c])
      peakCurrent[c] = irms[c];
    if (RMSPower[c] > peakPower[c])
      peakPower[c] = RMSPower[c];

    // Get time since last reading ...
    endMillis[c] = millis();
    unsigned long dTime = (endMillis[c] - startMillis[c]);
    startMillis[c] = endMillis[c];

    // How much power has been used ... ?
    double duration = ((double)dTime) / (60 * 60 * 1000.0); // Time in hours since last reading
    wattNow = RMSPower[c] * duration;                       // So many Wh have been used ...
    kilos[c] += wattNow / 1000;
    todayPower += wattNow;
    currentPower += RMSPower[c];
  }

#ifdef RX_DEBUG
  Serial.print("IRMS: ");
  for( int t=0; t<NR_OF_PHASES; t++ ) {
    Serial.print( irms[t]);
    Serial.print(" ");
  }
  Serial.println();
#endif

#ifdef USE_DISPLAY
  //Add stuff into the 'display buffer'
  if( m_display ) {
    DrawPower(todayPower);
    UpdateDisplay(false);
  }
#endif

#ifdef RX_DEBUG
//  Serial.println(buffer);
#ifdef USE_REMOTE_DBG
  debugV("%s", buffer);
#endif
#endif

  if (upd || iUpd > 60) { // Have updates to send ...
    char fBuf[128];
    int _n = sprintf(fBuf, "{");
    nReadings++;
    for(int i=0; i<NR_OF_PHASES; i++)
      _n += sprintf(fBuf+_n, "\"l%d\":%.2f,\"m%d\":%.2f,", i+1, irms[i], i+1, peakCurrent[i]);
    _n += sprintf(fBuf+_n, "\"power\":%.1f,", getCurrentPower());
    sprintf(fBuf+_n, "\"rssi\":%d,\"readings\":%d}", WiFi.RSSI(), nReadings);
    sendMsg("current", fBuf);
#ifdef USE_REMOTE_DBG
    debugV("Update: <%s>", fBuf);
#endif
    iUpd = 0;
  }

}
