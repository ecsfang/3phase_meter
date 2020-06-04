
// For the 0.96" OLED display
#include "3phase_utils.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     0 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void oled_setup()   {
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  OLED.setRotation(1);
  OLED.clearDisplay();

  //Add stuff into the 'display buffer'
  OLED.setTextColor(WHITE);
  OLED.setTextSize(2);
  OLED.setCursor(0,0);
  OLED.println("Power\nMeter");
  OLED.startscrollleft(0x00, 0x0F); //make display scroll 
  OLED.display(); //output 'display buffer' to screen  
  ClrDisplay();
  OLED.setRotation(0);
}

float scale(float f)
{
  return (WH*f)/IMAX;
}

int iMax[3] = {0,0,0};  // Used to draw max-indicator
int del[3];             // Delay to remove max indicator

void ClrDisplay(void)
{
  OLED.clearDisplay();
  OLED.setTextColor(WHITE);
  OLED.setTextSize(0);
  OLED.setCursor(0,0);
}

void UpdateDisplay(void)
{
  OLED.display(); //output 'display buffer' to screen  
}

void DrawBar(int r, float val)
{
    int pos = (int)scale(val);
    //draw the bar graph
    OLED.fillRect(Wm+pos, Wst+r*Woff, WH-pos, WB, BLACK);
    OLED.fillRect(Wm, Wst+r*Woff, pos, WB, WHITE);

    for (int i = 1; i < (pos/10)+1; i++) {
      OLED.fillRect(Wm + i*10, Wst+r*Woff, 2, WB, BLACK);
    }
    if( pos > iMax[r] ) {
      iMax[r] = pos;
      del[r] = 0;
    }

    // Draw a max-indicator on the display ...
#define BAR_DELAY 1
    if( iMax[r] > 0 ) {
      OLED.fillRect(Wm+iMax[r], Wst+r*Woff, 2, WB, WHITE);
      if( del[r] > BAR_DELAY )
        iMax[r] -= (del[r]-BAR_DELAY);
      del[r]++;
    }

    DrawPhase(val, r);
}

// Draw value under specified bar
void DrawPhase(double f, int i)
{
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(Wm-(6*4)-1, Wst+i*Woff+1);
  OLED.print(Format(f, 3, 1));
}

// Draw number at end of display
void DrawPower(double p)
{
  OLED.setRotation(1);
  OLED.setTextSize(2);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 114);
  OLED.print(Format(p, 5, 0));
  OLED.setRotation(0);
}

String Format(double val, int dec, int dig )
{
  // this is my simple way of formatting a number
  // data = Format(number, digits, decimals) when needed

  int addpad = 0;
  char sbuf[20];
  String fdata = (dtostrf(val, dec, dig, sbuf));
  int slen = fdata.length();
  for ( addpad = 1; addpad <= dec + dig - slen; addpad++) {
    fdata = " " + fdata;
  }
  return (fdata);
}

#define NR_OF_PHASES  3
#define SENSOR_PAR_CNT (NR_OF_PHASES*4+2)
#define BUF_LEN 1024

// Current values ...
extern double        irms[NR_OF_PHASES];
extern unsigned long RMSPower[NR_OF_PHASES];      // Current power (W)
extern double        peakCurrent[NR_OF_PHASES];   // Peak current (per day)
extern unsigned long peakPower[NR_OF_PHASES];     // Peak power (per day)
extern double        kilos[NR_OF_PHASES];         // Total kWh today (per phase)
extern unsigned long getTodayPower(void);         // Todays total
extern unsigned long getYesterdayPower(void);     // Yesterdays total
extern time_t  getNTPtime(void);

extern void sendMsg(const char *topic, const char *m);
extern void sendMsgF(const char *topic, double v);
extern void sendMsgI(const char *topic, int v);

extern bool  bSendStatus;

void sendStatus(void)
{
  char  values[BUF_LEN];
  char  dateBuf[32];
  int   n = 0;
  time_t local = 0;

  // Param 0
  n = sprintf(values, "VALUES:%ld", getTodayPower());
  // Param 1
  n += sprintf(values+n, ";%ld", getYesterdayPower());

  for( int c=0; c<NR_OF_PHASES; c++ ) {
    // CURRENT
    n += sprintf(values+n, ";");
    dtostrf(irms[c],1,1,values+n);
    n = strlen(values);
    // POWER
    n += sprintf(values+n, ";%d", RMSPower[c]);
    // TODAY
    n += sprintf(values+n, ";");
    dtostrf(kilos[c],1,1,values+n);
    n = strlen(values);
    // PEAK
    n += sprintf(values+n, ";");
    dtostrf(peakCurrent[c],1,1,values+n);
    n = strlen(values);
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
  json = R"({
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
    })";

  local = getNTPtime(); //timeClient.getEpochTime(); //now();
  sprintf(dateBuf, "\"%d.%02d.%02d %02d:%02d:%02d\"", year(local), month(local), day(local), hour(local), minute(local), second(local));

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
