/***************************************************************************
 POWER METER - with 3 phases and blink detection
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <Adafruit_ADS1015.h>
#include <ADS1115.h>

#include "EmonLib.h"

const int sclPin =            D1;
const int sdaPin =            D2;
const int blinkPin =          D3;

#define ADS0_A0 1
#define ADS0_A1 2
#define ADS0_A2 3
#define ADS0_A3 4

#define SEALEVELPRESSURE_HPA  (1013.25)
#define DISPLAY_ADDRESS       0x71
#ifdef BME280_ADDRESS
#undef BME280_ADDRESS
#define BME280_ADDRESS        0x76
#endif
#define ADS_ADDRESS           0x48

bool bHaveBME = false;
bool bHaveADS = false;
bool bHaveDisplay = false;

Adafruit_BME280   bme; // I2C
//Adafruit_ADS1015  ads;     /* Use thi for the 12-bit version */
ADS1115 adc(ADS1115_ADDRESS_ADDR_GND);

// Create  instances for each CT channel
EnergyMonitor ct1, ct2, ct3, ct4;

unsigned long delayTime = 1000;

void initAdc(ADS1115& adc) {
  Serial.println("Testing ADC connection...");
  Serial.println(adc.testConnection() ? "ADS1015 connection successful" : "ADS1015 connection failed");

  adc.initialize(); // initialize ADS1015 16 bit A/D chip
  //continuous mode. Data will always be ready
  adc.setMode(ADS1115_MODE_CONTINUOUS);
  //set the gain
  adc.setGain(ADS1115_PGA_6P144);
  //set to max rate
  adc.setRate(ADS1115_RATE_475);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("PowerMeter!"));

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(blinkPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(blinkPin), onPulse, FALLING);

    digitalWrite(LED_BUILTIN, HIGH);  // Off ...

    bool status;
  
    Wire.begin(sdaPin, sclPin);

    i2cScan();

    if( bHaveBME ) {
      // default settings
      // (you can also pass in a Wire library object like &Wire2)
      status = bme.begin(BME280_ADDRESS);  
      if (!status) {
          Serial.println("Could not find a valid BME280 sensor, check wiring!");
          while (1);
      }

      // weather monitoring
      Serial.println("-- Weather Station Scenario --");
      Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
      Serial.println("filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1, // temperature
                      Adafruit_BME280::SAMPLING_X1, // pressure
                      Adafruit_BME280::SAMPLING_X1, // humidity
                      Adafruit_BME280::FILTER_OFF   );
                      
      // suggested rate is 1/60Hz (1m)
      delayTime = 10000; // in milliseconds
    }
    
    if( bHaveDisplay ) {
      //Send the reset command to the display - this forces the cursor to return to the beginning of the display
      Wire.beginTransmission(DISPLAY_ADDRESS);
      Wire.write('v');
      Wire.endTransmission();
    }
    
    if( bHaveADS ) {
      initAdc(adc);
//      ads.begin();
    }

    Serial.println();

    testADC();

    ct1.inputPinReader = ads1115PinReader; // Replace the default pin reader with the customized ads pin reader
    ct2.inputPinReader = ads1115PinReader;
    ct3.inputPinReader = ads1115PinReader;

    // Calibration factor = CT ratio / burden resistance
    // the current constant is the value of current you want to read when 1 V is produced at the analogue input
    ct1.current(ADS0_A0, 30);
    ct2.current(ADS0_A1, 30);
    ct3.current(ADS0_A2, 30);

    // (ADC input, calibration, phase_shift)
    //ct1.voltage(ADS1_A0, 240, 1.2);
    //ct2.voltage(ADS1_A0, 240, 1.2);
    //ct3.voltage(ADS1_A0, 240, 1.2);
}

bool bBlink = false;
int blinkCnt = 0;

void loop() { 
 /*
  if( bBlink ) {
    Serial.print("Blinks: ");
    Serial.println(blinkCnt);
    bBlink = false;
    if( bHaveDisplay ) 
      i2cSendValue(blinkCnt); // Send to display ...
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if( bHaveBME )
    printValues();
  if( bHaveADS )
    checkADC();
  delay(delayTime);
*/
  // Calculate all. No.of crossings, time-out
  //ct4.calcVI(20,2000);

  // Print power
  Serial.println("READ BEGIN");
  unsigned long start = millis();

  Serial.print("ct1: ");
  Serial.println( ct1.calcIrms(1480) );

  Serial.print("ct2: ");
  Serial.println( ct2.calcIrms(1480) );

  Serial.print("ct3: ");
  Serial.println( ct3.calcIrms(1480) );

  Serial.print("Vrms");
  Serial.println(ct1.Vrms);

  Serial.print("took ");Serial.print(millis() - start);Serial.println("ms");
  Serial.println("READ END");

  delay(2000);  
}

// The interrupt routine
void onPulse()
{
  digitalWrite(LED_BUILTIN, LOW);
  blinkCnt++;
  bBlink = true;
}

void printValues() {
  char buf[80];

    sprintf(buf, "Temperature = %.1f *C", bme.readTemperature());
    Serial.println(buf);

    sprintf(buf, "Pressure = %.1f hPa", bme.readPressure() / 100.0F);
    Serial.println(buf);


    sprintf(buf, "Approx. Altitude = %.0f m", bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(buf);

    sprintf(buf, "Humidity = %.1f %%", bme.readHumidity());
    Serial.println(buf);

    Serial.print("Blink = ");
    Serial.println(blinkCnt);

    Serial.println();
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
      switch( address ) {
        case BME280_ADDRESS:
          Serial.print(" [BME280]");
          bHaveBME = true;
          break;
        case DISPLAY_ADDRESS:
          Serial.print(" [7-SEGMENT]");
          bHaveDisplay = true;
          break;
        case ADS_ADDRESS:
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


  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
void checkADC(void) 
{
  int16_t adc0, adc1, adc2, adc3;

  adc0 = adc.getConversionP0GND(); //ads.readADC_SingleEnded(0);
  adc1 = adc.getConversionP1GND(); //ads.readADC_SingleEnded(1);
  adc2 = adc.getConversionP2GND(); //ads.readADC_SingleEnded(2);
  adc3 = adc.getConversionP3GND(); //ads.readADC_SingleEnded(3);
  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  Serial.println(" ");
  
  delay(1000);
}

unsigned long startMillis=0;
unsigned long endMillis=0;
#define TST_CNT 100

void testADC(void)
{

  int16_t adc0, adc1, adc2, adc3;

  startMillis = millis();

  for(int i =0; i<TST_CNT; i++) {
    adc0 = adc.getConversionP0GND(); //ads.readADC_SingleEnded(0);
    adc1 = adc.getConversionP0GND(); //ads.readADC_SingleEnded(1);
    adc2 = adc.getConversionP0GND(); //ads.readADC_SingleEnded(2);
    adc3 = adc.getConversionP0GND(); //ads.readADC_SingleEnded(3);
  }
  endMillis = millis() - startMillis;

  Serial.print(TST_CNT*4); Serial.print(" conversations took ");
  Serial.print(endMillis); Serial.println(" ms");
  Serial.print((TST_CNT*4.0)*1000/endMillis); Serial.println(" conv/s");
  Serial.println("");  
}
//Given a number, i2cSendValue chops up an integer into four values and sends them out over I2C
void i2cSendValue(int tempCycles)
{
  Wire.beginTransmission(DISPLAY_ADDRESS); // transmit to device at new I2C address
  Wire.write(tempCycles / 1000); //Send the left most digit
  tempCycles %= 1000; //Now remove the left most digit from the number we want to display
  Wire.write(tempCycles / 100);
  tempCycles %= 100;
  Wire.write(tempCycles / 10);
  tempCycles %= 10;
  Wire.write(tempCycles); //Send the right most digit
  Wire.endTransmission(); //Stop I2C transmission
}

// Make a callback method for reading the pin value from the ADS instance
int ads1115PinReader(int addr){
  int16_t tmp;
  switch (addr) {
    case ADS0_A0: tmp = adc.getConversionP0GND(); break;
    case ADS0_A1: tmp = adc.getConversionP1GND(); break;
    case ADS0_A2: tmp = adc.getConversionP2GND(); break;
    case ADS0_A3: tmp = adc.getConversionP3GND(); break;
  }
  //tmp = analogRead(addr);
  //Serial.print(tmp); Serial.print(" -- "); Serial.println(tmp>>6);
  //Serial.print(addr); Serial.print(": "); Serial.println(tmp*ADS1115_MV_6P144 / 5);

  //calculation:
  // tmp * ADS1115_MV_6P144 returns a voltage measurement between 0 and 5v
  // emonlib expect a value between 0 an 1024, so convert
  // TODO: we're loosing precision here as it's converted to a 10 bit value. fix this.
  return (tmp * ADS1115_MV_6P144 * 1024) / 5000;
  //return tmp*ADS1115_MV_6P144;
  //return tmp*ADS1115_MV_1P024;

}
