//#include <Adafruit_ADS1015.h>
#include <ADS1115.h>

#define ADS0_A0 1
#define ADS0_A1 2
#define ADS0_A2 3
#define ADS0_A3 4



void initAdc(ADS1115& adc);
int ads1115PinReader(int addr);

