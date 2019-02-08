//#define USE_ADS1015
#define USE_MCP3008

#ifdef USE_ADS1015
#include <Adafruit_Sensor.h>
#include <ADS1115.h>
#endif

#ifdef USE_MCP3008
#include <MCP3XXX.h>
#endif

#ifdef USE_ADS1015
#define ADS_A0 1
#define ADS_A1 2
#define ADS_A2 3
#define ADS_A3 4

void initAdc(void);
int adcPinReader(int addr);
#endif

#ifdef USE_MCP3008
#define ADS_A0 5
#define ADS_A1 6
#define ADS_A2 7
//#define ADS_A3 3
void initAdc(void);
int adcPinReader(int addr);
void testADC(void);
#endif
