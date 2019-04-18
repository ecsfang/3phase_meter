
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// For timekeeping with correct daylight savings ...
#include <NTPClient.h>
#include <time.h>                 // time() ctime()
#include <sys/time.h>             // struct timeval
#include <coredecls.h>            // settimeofday_cb()
#include <Timezone.h>             // https://github.com/JChristensen/Timezone


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS (0x78>>1)

#define IMAX  30    // Maximum current (max of drawn bar)

#define Wm    43    // Start of bar (left border of bar)
#define WB    10    // Width of bar (height)
#define WH    (SCREEN_WIDTH-Wm) // Total length of bar

#define Wst   3     // Top screen to bar
#define Woff  24    // Offset to next bar

void sendStatus(void);

void oled_setup(void);
void DrawPhase(double f, int i);
// Draw number at end of display
void DrawPower(double p);
void DrawBar(int r, float val);
float scale(float f);
String Format(double val, int dec, int dig );
