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
#define ADC_CH0 1
#define ADC_CH1 2
#define ADC_CH2 3
#define ADC_CH3 4

void initAdc(void);
int adcPinReader(int addr);
#endif

#ifdef USE_MCP3008
#define ADC_CH0 0
#define ADC_CH1 1
#define ADC_CH2 2
#define ADC_CH3 3
#define ADC_CH4 4
#define ADC_CH5 5
#define ADC_CH6 6
#define ADC_CH7 7
void initAdc(void);
int adcPinReader(int addr);
void testADC(void);
#endif
