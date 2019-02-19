#include "3phase_adc.h"

#ifdef USE_ADS1015
ADS1115 adc(ADS1115_ADDRESS_ADDR_GND);
bool bHaveADS;
void initAdc(void) {
  i2cScan();
  if( bHaveADS ) {
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

void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      switch ( address ) {
        case ADS1115_ADDRESS_ADDR_GND:
          Serial.print(" [ADS]");
          bHaveADS = true;
          break;
      }
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.print("Done! ");
    Serial.print(nDevices);
    Serial.println(" devices found.\n");
  }

  delay(2000);
}
#endif

#ifdef USE_MCP3008
MCP3008 mcp;

void initAdc(void)
{
  mcp.begin();
}

bool bNeg[3];

#define USE_DIFFERENTIAL

// Make a callback method for reading the pin value from the ADS instance
int adcPinReader(int addr)
{
  int32_t tmp;

#ifdef USE_DIFFERENTIAL
  int chan = addr*2;

  if( bNeg[addr] )
    tmp = -mcp.analogReadDifferential(chan+1);
  else
    tmp = mcp.analogReadDifferential(chan);
  if (tmp == 0)
    bNeg[addr] != bNeg[addr];
#else
    tmp = mcp.analogRead(addr);
#endif

  // 333 ohm 0.5mA/A -> 30A = 15mA -> 5V -> tmp = +/- 5v

  //calculation:
  // tmp * ADS1115_MV_6P144 returns a voltage measurement between 0 and 5v
  // emonlib expect a value between 0 an 1024, so convert
  // TODO: we're loosing precision here as it's converted to a 10 bit value. fix this.
//  return (tmp*adc.getMvPerCount()*1024)/3300; // 0-1024 (where 1024 = 3.3V)
  return tmp*1110/2000;
}
#endif
