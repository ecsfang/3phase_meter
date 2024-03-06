
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// For timekeeping with correct daylight savings ...
#include <NTPClient.h>
#include <time.h>                 // time() ctime()
#include <sys/time.h>             // struct timeval
#include <coredecls.h>            // settimeofday_cb()
#include <Timezone.h>             // https://github.com/JChristensen/Timezone

//#define HOUSE
//#define HEATER
#define SPA
//#define KITCHEN

#define USE_REMOTE_DBG

#ifdef HOUSE
#define METER "House"
#endif
#ifdef HEATER
#define METER "Heater"
#endif
#ifdef SPA
#define METER "Spa"
#endif
#ifdef KITCHEN
#define METER "Kitchen"
#endif

#ifdef HOUSE
#define MESSAGE "housePower"  // Default message
#define SCT_013_000        // The sensor used
#define NR_OF_PHASES 3      // Number of phases to watch
#endif
#ifdef HEATER
#define MESSAGE "heaterPower" // Default message
#define SCT_013_000        // The sensor used
#define USE_CORRECTION
#define USE_DISPLAY
#define NR_OF_PHASES 3      // Number of phases to watch
#endif
#ifdef KITCHEN
#define MESSAGE "kitchenPower" // Default message
#define SCT_013_030        // The sensor used
#define NR_OF_PHASES 4      // Number of phases to watch
#define USE_CORRECTION
#endif
#ifdef SPA
#define MESSAGE "spaPower"    // Default message
#define SCT_013_000        // The sensor used
#define NR_OF_PHASES 3      // Number of phases to watch
#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS (0x78>>1)

#define IMAX  30    // Maximum current (max of drawn bar)

#define Wm    43    // Start of bar (left border of bar)
#define WB    10    // Width of bar (height)
#define WH    (SCREEN_WIDTH-Wm) // Total length of bar

#define Wst   2     // Top screen to bar
#define Woff  24    // Offset to next bar

#ifdef USE_REMOTE_DBG
#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug
extern RemoteDebug Debug;
#endif

void sendStatus(void);

void oled_setup(void);
void ClrDisplay(void);
void UpdateDisplay(void);
void DrawPhase(double f, int i);
// Draw number at end of display
void DrawPower(double p);
void DrawBar(int r, float val);
float scale(float f);
String Format(double val, int dec, int dig );

// Current values ...
extern double         irms[NR_OF_PHASES];
extern unsigned long  RMSPower[NR_OF_PHASES];     // Current power (W)
extern double         peakCurrent[NR_OF_PHASES];  // Peak current (per day)
extern unsigned long  peakPower[NR_OF_PHASES];    // Peak power (per day)
extern double         kilos[NR_OF_PHASES];        // Total kWh today (per phase)
extern double         getCurrentPower(void);      // Since last reading
extern unsigned long  getTodayPower(void);        // Todays total
extern unsigned long  getYesterdayPower(void);    // Yesterdays total
extern time_t         getNTPtime(void);

extern char *ip(void);

extern void sendMsg(const char *topic, const char *m);
extern void sendMsgF(const char *topic, double v);
extern void sendMsgI(const char *topic, int v);

extern bool  bSendStatus;
