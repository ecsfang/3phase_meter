//#define ADS1015
#define MCP3008

#ifdef ADS1015
#include <Adafruit_Sensor.h>
#include <ADS1115.h>
#endif

#ifdef MCP3008
#include <MCP3XXX.h>
#endif

#ifdef ADS1015
#define ADS0_A0 1
#define ADS0_A1 2
#define ADS0_A2 3
#define ADS0_A3 4

void initAdc(ADS1115& adc);
int ads1115PinReader(int addr);
#endif

#ifdef MCP3008
void initAdc(void);
int mcp3008PinReader(int addr);
#endif
