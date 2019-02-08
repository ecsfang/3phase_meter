#include "3phase_adc.h"

#ifdef USE_ADS1015
ADS1115 adc(ADS1115_ADDRESS_ADDR_GND);

void initAdc(void) {
  Serial.println("Testing ADC connection...");
  Serial.println(adc.testConnection() ? "ADS1015 connection successful" : "ADS1015 connection failed");

  adc.initialize(); // initialize ADS1015 12 bit A/D chip
  //continuous mode. Data will always be ready
  adc.setMode(ADS1115_MODE_CONTINUOUS);
  //set the gain
  adc.setGain(ADS1115_PGA_4P096);
  //set to max rate
  adc.setRate(ADS1115_RATE_475);
}

// Make a callback method for reading the pin value from the ADS instance
int adcPinReader(int addr)
{
  int32_t tmp;

  switch (addr) {
    case ADS_A0: tmp = adc.getConversionP0GND(); break;
    case ADS_A1: tmp = adc.getConversionP1GND(); break;
    case ADS_A2: tmp = adc.getConversionP3GND(); break;
    case ADS_A3: tmp = adc.getConversionP2GND(); break;
  }
  //Serial.print(tmp); Serial.print(" -- "); Serial.println(tmp>>6);
  //Serial.print(addr); Serial.print(": "); Serial.println(tmp*ADS1115_MV_6P144 / 5);

  //calculation:
  // tmp * ADS1115_MV_6P144 returns a voltage measurement between 0 and 5v
  // emonlib expect a value between 0 an 1024, so convert
  // TODO: we're loosing precision here as it's converted to a 10 bit value. fix this.
  return (tmp*adc.getMvPerCount()*1024)/3300; // 0-1024 (where 1024 = 3.3V)
}
#endif

#ifdef USE_MCP3008
MCP3008 mcp;

void initAdc(void)
{
  mcp.begin();
}

bool bNeg[3];

// Make a callback method for reading the pin value from the ADS instance
int adcPinReader(int addr)
{
  int32_t tmp;

  tmp = mcp.analogRead(addr);

  // tmp = (5V/2 +/- 1V -> ca 1.5 -> 3.5 V)
  // --> 310 -> 715
  //calculation:
  // tmp * ADS1115_MV_6P144 returns a voltage measurement between 0 and 5v
  // emonlib expect a value between 0 an 1024, so convert
  // TODO: we're loosing precision here as it's converted to a 10 bit value. fix this.
//  return (tmp*adc.getMvPerCount()*1024)/3300; // 0-1024 (where 1024 = 3.3V)
  return tmp; // 1024 = 5V
}

// Make a callback method for reading the pin value from the ADS instance
int adcPinReaderDiff(int addr)
{
  int32_t tmp;

  int chan = addr*2;

  if( bNeg[addr] )
    tmp = -mcp.analogReadDifferential(chan+1);
  else
    tmp = mcp.analogReadDifferential(chan);

  if (tmp == 0)
    bNeg[addr] != bNeg[addr];

  // 333 ohm 0.5mA/A -> 30A = 15mA -> 5V -> tmp = +/- 5v

  //calculation:
  // tmp * ADS1115_MV_6P144 returns a voltage measurement between 0 and 5v
  // emonlib expect a value between 0 an 1024, so convert
  // TODO: we're loosing precision here as it's converted to a 10 bit value. fix this.
//  return (tmp*adc.getMvPerCount()*1024)/3300; // 0-1024 (where 1024 = 3.3V)
  return tmp*1110/2000;
}

void testADC(void)
{
  for (size_t i = 0; i < mcp.numChannels(); ++i)
  {
    Serial.print(mcp.analogRead(i));
    
    if (i == mcp.numChannels() - 1)
    {
      Serial.println();
    }
    else
    {
      Serial.print(",");
    }
  }

  delay(10);
}

#endif
