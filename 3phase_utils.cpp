
#include "3phase_utils.h"

// For the 0.96" OLED display
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     0 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void oled_setup()   {
#ifdef USE_DISPLAY
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  OLED.setRotation(1);
  ClrDisplay();

  //Add stuff into the 'display buffer'
  OLED.setTextSize(2);
<<<<<<< Updated upstream
  OLED.println("Power\nMeter");
  UpdateDisplay(true); //output 'display buffer' to screen  
=======
  OLED.setCursor(0,0);
#ifdef DISP_MSG
  OLED.print(DISP_MSG);
  OLED.println("\nMeter");
#else
  OLED.println("Power\nMeter");
#endif
OLED.startscrollleft(0x00, 0x0F); //make display scroll 
  OLED.display(); //output 'display buffer' to screen  
>>>>>>> Stashed changes
  ClrDisplay();
#ifdef HEATER
  OLED.setRotation(2);
#else
  OLED.setRotation(0);
#endif
}

float scale(float f)
{
  return (WH*f)/IMAX;
}

int iMax[3] = {0,0,0};  // Used to draw max-indicator
int del[3];             // Delay to remove max indicator

void ClrDisplay(void)
{
#ifdef USE_DISPLAY
  OLED.clearDisplay();
  OLED.setTextColor(WHITE);
  OLED.setTextSize(1);
  OLED.setCursor(0,0);
#endif
}


void UpdateDisplay(bool bScroll)
{
  if( !bScroll )
    OLED.stopscroll();
  OLED.display(); //output 'display buffer' to screen
  if( bScroll )
    OLED.startscrollleft(0x00, 0x0F); //make display scroll
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
#ifdef HEATER
  OLED.setRotation(2);
#else
  OLED.setRotation(0);
#endif
//  OLED.setRotation(0);
}

void DispText(int x, int y, char *txt)
{
#ifdef USE_DISPLAY
  OLED.setCursor(x, y);
  OLED.print(txt);
#endif
}

#ifdef USE_DISPLAY
void DispError(char *error)
{
  ClrDisplay();
  OLED.clearDisplay();
  //Add stuff into the 'display buffer'
  OLED.setCursor(0, 10);
  OLED.print(error);
  ClrDisplay(true);
}
#endif

// Format a string right justified
// p = 3.14
// Format(p, 5, 1) --> "  3.1"
String Format(double val, int dec, int dig )
{
  // this is my simple way of formatting a number
  // data = Format(number, digits, decimals) when needed
  // val - value to be printed
  // dec - number of total characters in string
  // dig - number of decimal digits

  int addpad = 0;
  char sbuf[20];
  String fdata = (dtostrf(val, dec, dig, sbuf));
  int slen = fdata.length();
  for ( addpad = 1; addpad <= dec + dig - slen; addpad++) {
    fdata = " " + fdata;
  }
  return (fdata);
}

#define SENSOR_PAR_CNT (NR_OF_PHASES*4+2+1)
#define BUF_LEN 1024

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

  // Param 13
  n += sprintf(values+n, ";%.1f", getCurrentPower());

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
#if NR_OF_PHASES == 3
  String json;
#if NR_OF_PHASES == 4
  json = R"({
    "Time": $TIME,
    "IP": $IP,
    "ENERGY": {
      "Now": $NOW,
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
      },
      "Phase4": {
        "Today": $TODAY_4,
        "Current": $CURRENT_4,
        "Power": $POWER_4,
        "Peak": $PEAK_4
      }
    }
  })";
#endif
#if NR_OF_PHASES == 3
  json = R"({
    "Time": $TIME,
    "IP": $IP,
    "ENERGY": {
      "Now": $NOW,
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
<<<<<<< Updated upstream
    }
  })";
#endif
#if NR_OF_PHASES == 4
  String json;
  json = R"({
    "Time": $TIME,
    "IP": $IP,
    "ENERGY": {
      "Now": $NOW,
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
      },
      "Phase4": {
        "Today": $TODAY_4,
        "Current": $CURRENT_4,
        "Power": $POWER_4,
        "Peak": $PEAK_4
      }
=======
>>>>>>> Stashed changes
    }
  })";
#endif

  // Get current time
  local = getNTPtime();
  // Format the date and time
  sprintf(dateBuf, "\"%d.%02d.%02d %02d:%02d:%02d\"",
        year(local), month(local), day(local),
        hour(local), minute(local), second(local));
  
  n = 0;
  json.replace("$TIME",       dateBuf);
  json.replace("$IP",         ip());
  json.replace("$TOTAL",      par[n++]);
  json.replace("$YDAY",       par[n++]);

  json.replace("$CURRENT_1",  par[n++]);
  json.replace("$POWER_1",    par[n++]);
  json.replace("$TODAY_1",    par[n++]);
  json.replace("$PEAK_1",     par[n++]);

  json.replace("$CURRENT_2",  par[n++]);
  json.replace("$POWER_2",    par[n++]);
  json.replace("$TODAY_2",    par[n++]);
  json.replace("$PEAK_2",     par[n++]);

  json.replace("$CURRENT_3",  par[n++]);
  json.replace("$POWER_3",    par[n++]);
  json.replace("$TODAY_3",    par[n++]);
  json.replace("$PEAK_3",     par[n++]);

#if NR_OF_PHASES == 4
  json.replace("$CURRENT_4",  par[n++]);
  json.replace("$POWER_4",    par[n++]);
  json.replace("$TODAY_4",    par[n++]);
  json.replace("$PEAK_4",     par[n++]);
#endif

  json.replace("$NOW",        par[n++]);

#ifdef USE_REMOTE_DBG
  Debug.print("json: ");
  Debug.println(json.c_str());
#endif

  sendMsg("status", json.c_str());
  bSendStatus = false;
}
