// Element Pro Firmware v0.4
// Modified: 07/10/2019

// Developed by Akram Ali
// github.com/akstudios

#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <RFM69.h>              // https://github.com/LowPowerLab/RFM69
#include <SPIFlash.h>           // https://github.com/lowpowerlab/spiflash
#include <Adafruit_ADS1015.h>   // https://github.com/adafruit/Adafruit_ADS1X15
#include <Adafruit_SGP30.h>     // https://github.com/adafruit/Adafruit_SGP30
#include <Adafruit_SHT31.h>     // https://github.com/adafruit/Adafruit_SHT31
#include <Adafruit_BMP3XX.h>    // https://github.com/adafruit/Adafruit_BMP3XX
#include <Adafruit_Sensor.h>    // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_TSL2591.h>   // https://github.com/adafruit/Adafruit_TSL2591_Library
#include <Adafruit_NeoPixel.h>  // https://github.com/adafruit/Adafruit_NeoPixel
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// define node parameters
//#define NODEID      109
uint16_t NODEID =     999; // same as above, but supports 10bit addresses (up to 1023 node IDs)
#define GATEWAYID     1
#define NETWORKID     1
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "Tt-Mh=SQ#dn#JY3_" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define PIN           6 // NeoPixel driver pin
#define FLASH_SS      5 // and FLASH SS on D5
#define PM_SET        9 // controls PMS7003 fan + sensor power

// define objects
RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SGP30 sgp;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);  // number of pixels, digital pin, pixel flags
Adafruit_ADS1115 ads;
Adafruit_BMP3XX bmp;

// define PMS7003 global variables
SoftwareSerial pm_serial(A1, 9); // RX, TX
char buf[31];
long CF1PM01Value,CF1PM25Value,CF1PM10Value,atmPM01Value,atmPM25Value,atmPM10Value,Partcount0_3,Partcount0_5,Partcount1_0,Partcount2_5,Partcount5_0,Partcount10;

// define S8 CO2 global variables
SoftwareSerial co2_serial(7,8);  //Sets up a virtual serial port
                                    //Using pin 12 for Rx and pin 13 for Tx
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response
//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;

// define other global variables
long lux;
float bar;
float sound;
int ppm, _ppm;
float co2;
uint16_t tvoc;
float adc16;
adsGain_t gain[6] = {GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN};  // create an array of type adsGain_t, which is a struct in the Adafruit_ADS1015.h library
int gn = 5;      // set gain to 16x to begin with
#define SEALEVELPRESSURE_HPA (1013.25)

char dataPacket[150];

ISR(WDT_vect)  // Interrupt Service Routine for WatchDog Timer
{
  wdt_disable();  // disable watchdog
}


void ISR_button()  // Interrupt Service Routine for button press
{
  for (int i = 0; i <= 255; i++)
  {
      colorWipe(strip.Color(0,0,i), 1);  // blue
  }
  for (int i = 255; i > 0; i--)
  {
      colorWipe(strip.Color(0,0,i), 2);  // blue
  }
  colorWipe(strip.Color(0, 0, 0), 1); // turn pixel off
  strip.show();
}


void setup()
{
  pinMode(10, OUTPUT); // Radio SS pin set as output

  Serial.begin(115200);
  Serial.println("Setup");

  //pinMode(3, INPUT);
  pinMode(PM_SET, OUTPUT);
  digitalWrite(PM_SET, HIGH);
  attachInterrupt(1, ISR_button, FALLING);  // enable hardware interrupt on pin 3 when pin goes from HIGH to LOW

  flash.initialize();
  co2_serial.begin(9600);
  ads.begin();
  ads.setGain(gain[gn]);
  sgp.begin();
  bmp.begin();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  pm_serial.begin(9600);   // initialize PMS7003 in UART mode
  digitalWrite(PM_SET, LOW);  // turn PM sensor off

//  while(mySerial2.available() > 0)  // clear out buffer
//    char x = mySerial2.read();
//  delay(1);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);

  strip.begin(); // initialize neo pixels
  strip.show(); // Initialize all pixels to 'off'

  Serial.println("Ready");
  delay(10);
}


void sleep()
{
  Serial.flush(); // empty the send buffer, before continue with; going to sleep

  flash.sleep();
  radio.sleep();

  cli();          // stop interrupts
  MCUSR = 0;
  WDTCSR  = (1<<WDCE | 1<<WDE);     // watchdog change enable
  WDTCSR  = 1<<WDIE | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0); // set  prescaler to 8 second
  sei();  // enable global interrupts

  byte _ADCSRA = ADCSRA;  // save ADC state
  ADCSRA &= ~(1 << ADEN);

  asm("wdr");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();

  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();

  sleep_disable();
  sei();

  ADCSRA = _ADCSRA; // restore ADC state (enable ADC)
  delay(1);
}


void loop()
{
  sleep();
  readSensors();

  Serial.println(dataPacket);
  delay(50);

  // send datapacket
  radio.sendWithRetry(GATEWAYID, dataPacket, strlen(dataPacket), 5, 100);  // send data, retry 5 times with delay of 100ms between each retry
  dataPacket[0] = (char)0; // clearing first byte of char array clears the array

  colorWipe(strip.Color(0, 255, 0), 10); // Green
  strip.show();

  for (int i = 0; i <= 255; i++)
  {
      colorWipe(strip.Color(0,i,0), 1);
  }
  for (int i = 255; i > 0; i--)
  {
      colorWipe(strip.Color(0,i,0), 2);
  }
  colorWipe(strip.Color(0, 0, 0), 1); // turn pixel off
  strip.show();
}


void readSensors()
{
  // T/RH - SHT31
  sht31.begin(0x44);
  float temp = sht31.readTemperature();
  float rh = sht31.readHumidity();  // taking temp reading is important for rh reading

  // ADS1115 16-bit ADC
  ads.setGain(gain[1]);  // this is only for temperature sensor
  adc16 = samples(0);   // get avg ADC value from channel 0 
  float R = resistance(adc16, 10000); // Replace 10,000 ohm with the actual resistance of the resistor measured using a multimeter (e.g. 9880 ohm)
  float a = steinhart(R);  // get air temperature from thermistor using the custom Steinhart-hart equation by US sensors

  // external voltage - 10K-10K voltage divider
  adc16 = ads_autogain(3);   // get avg ADC value from channel 3 
  float volt = voltage(adc16, gn);  // convert ADC value to a voltage reading based on the gain
  
  // Light Intensity - TSL2591
  tsl.begin();
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  sensors_event_t event;
  tsl.getEvent(&event);
  if ((event.light == 0) | (event.light > 4294966000.0) | (event.light <-4294966000.0))
  {
    lux = 0;  // invalid value; replace with 'NAN' if needed
  }
  else
  {
    lux = event.light;
  }
  
  //S8 CO2
  co2 = getCO2(readCO2);
  
  // BMP388 air pressure sensor
  bmp.performReading();
  bar = bmp.pressure / 100.0;

  // SGP30 VOC readings
  sgp.IAQmeasure();
  tvoc = sgp.TVOC;

  // PMS7003
  digitalWrite(PM_SET, HIGH); // turn PM sensor on
  delay(3000);  // wait a few seconds for air to flow through PM sensor
  long pm;
  pm_serial.listen();
  delay(1);
  if(pm_serial.available()>0)
    pm_serial.read();
  pm_serial.readBytes(buf, 31);
  if (pm_serial.find(0x42))
  {
      pm_serial.readBytes(buf, 31);
      get_pm_data(buf);
  }
  delay(1);
  pm = atmPM25Value;
  digitalWrite(PM_SET, LOW);  // turn PM sensor off


  // define character arrays for all variables
  char _i[3];
  char _t[7];
  char _h[7];
  char _a[7];
  char _c[7];
  char _o[7];
  char _g[7];
  char _l[7];
  char _n[7];
  char _s[7];
  char _v[7];
  char _z[7];
  char _p[7];

  // convert all flaoting point and integer variables into character arrays
  dtostrf(NODEID, 1, 0, _i);
  dtostrf(temp, 3, 2, _t);  // this function converts float into char array. 3 is minimum width, 2 is decimal precision
  dtostrf(rh, 3, 2, _h);
  dtostrf(a, 3, 2, _a);
  dtostrf(co2, 1, 0, _c);
  dtostrf(bar, 3, 2, _g);
  dtostrf(lux, 1, 0, _l);
  dtostrf(tvoc, 1, 0, _v);
  dtostrf(volt, 3, 2, _z);
  dtostrf(pm, 1, 0, _p);
  delay(50);

  dataPacket[0] = 0;  // first value of dataPacket should be a 0

  // create datapacket by combining all character arrays into a large character array
  strcat(dataPacket, "i:");
  strcat(dataPacket, _i);
  strcat(dataPacket, ",t:");
  strcat(dataPacket, _t);
  strcat(dataPacket, ",h:");
  strcat(dataPacket, _h);
  strcat(dataPacket, ",a:");
  strcat(dataPacket, _a);
  strcat(dataPacket, ",c:");
  strcat(dataPacket, _c);
  strcat(dataPacket, ",g:");
  strcat(dataPacket, _g);
  strcat(dataPacket, ",l:");
  strcat(dataPacket, _l);
  strcat(dataPacket, ",p:");
  strcat(dataPacket, _p);
  strcat(dataPacket, ",z:");
  strcat(dataPacket, _z);
  strcat(dataPacket, ",v:");
  strcat(dataPacket, _v);
  delay(50);
}


void get_pm_data(unsigned char b[])
{
  int receiveSum = 0;
  for (int i = 0; i < (31 - 2); i++)
  {
    receiveSum = receiveSum + b[i];
  }
  receiveSum = receiveSum + 0x42;
  receiveSum == ((b[31 - 2] << 8) + b[31 - 1]);

  if(b[0] == 0x4D && receiveSum)
  {
    CF1PM01Value = (b[9] << 8) + b[10]; //PM1.0 CF1 value of the air detector module
    CF1PM25Value = (b[5] << 8) + b[6]; //PM2.5 CF1 value of the air detector module
    CF1PM10Value = (b[7] << 8) + b[8]; //PM10 CF1 value of the air detector module

    atmPM01Value = (b[3] << 8) + b[4]; //PM1.0 atmospheric value of the air detector module
    atmPM25Value = (b[11] << 8) + b[12]; //PM2.5 atmospheric value of the air detector module
    atmPM10Value = (b[13] << 8) + b[14]; //PM10 atmospheric value of the air detector module

    Partcount0_3 = (b[15] << 8) + b[16]; //count particules > 0.3 µm
    Partcount0_5 = (b[17] << 8) + b[18]; //count particules > 0.5 µm
    Partcount1_0 = (b[19] << 8) + b[20]; //count particules > 1.0 µm
    Partcount2_5 = (b[21] << 8) + b[22]; //count particules > 2.5 µm
    Partcount5_0 = (b[23] << 8) + b[24]; //count particules > 5.0 µm
    Partcount10 = (b[25] << 8) + b[26]; //count particules > 10.0 µm
  }
}

float ads_autogain(int pin)
{
  float adc=0.0;
  int timeout = 1000;
  long t1 = millis();
  while(1) // this function constantly adjusts the gain to an optimum level
  {
    long t2 = millis();
    adc = samples(pin);   // get avg ADC value from channel pin

    if(adc >= 30000 && gn > 0)  // if ADC is getting pegged at maximum value and is not the widest voltage range already, reduce the gain
    {
      gn--;
      ads.setGain(gain[gn]);
    }
    else if(adc <= 5000 && gn < 5)  // if ADC is reading very low values and is not the lowest voltage range already, increase the gain
    {
      gn++;
      ads.setGain(gain[gn]);
    }
    else
      break;

    if(abs(t2-t1) > timeout)   // if nothing is connected, it'll be stuck in while forever, so this is precaution
      break;
  }
  adc = samples(pin);   // get final avg ADC value from channel pin
  return adc;
}


// Perform multiple iterations to get higher accuracy ADC values (reduce noise) ******************************************
float samples(int pin)
{
  float n=5.0;  // number of iterations to perform
  float sum=0.0;  //store sum as a 32-bit number
  for(int i=0;i<n;i++)
  {
    float value = ads.readADC_SingleEnded(pin);
    sum = sum + value;
    delay(1); // makes readings slower - probably don't need this delay, but ¯\_(ツ)_/¯
  }
  float average = sum/n;   //store average as a 32-bit number with decimal accuracy
  return average;
}


// Get resistance ****************************************************************
float resistance(float adc, int true_R)
{
  //float R = true_R/(1023.0/adc-1.0);   // convert 10-bit reading into resistance
  float ADCvalue = adc*(8.192/3.3);  // Vcc = 8.192 on GAIN_ONE setting, Arduino Vcc = 3.3V in this case
  float R = true_R/(65535/ADCvalue-1);  // 65535 refers to 16-bit number
  return R;
}

// Get temperature from Steinhart equation (US sensors thermistor, 10K, B = 3892) *****************************************
float steinhart(float R)
{
  float A = 0.00113929600457259;
  float B = 0.000231949467390149;
  float C = 0.000000105992476218967;
  float D = -0.0000000000667898975192618;
  float E = log(R);
  
  float T = 1/(A + (B*E) + (C*(E*E*E)) + (D*(E*E*E*E*E)));
  T -= 273.15;
  return T;
}

// Get temperature from Steinhart equation (10K Precision Epoxy Thermistor - 3950 NTC) *****************
float steinhart2(float R)
{
  float steinhart;
  steinhart = R / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950.0;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25.0 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  return steinhart;
}

  
// Get CO2 reading from S8 sensor ****************************************************************
long getCO2(byte packet[])
{
  co2_serial.listen();
  while(!co2_serial.available())  //keep sending request until we start to get a response
  {
    co2_serial.write(readCO2,7);
    delay(10);
  }
  
  int timeout=0;  //set a timeoute counter
  while(co2_serial.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;  
    if(timeout > 10)    //if it takes to long there was probably an error
      {
        while(co2_serial.available())  //flush whatever we have
          co2_serial.read();
          
          break;                        //exit and try again
      }
      delay(10);
  }
  
  for (int i=0; i < 7; i++)
  {
    response[i] = co2_serial.read();
  }
  
  int high = response[3];                        //high byte for value is 4th byte in packet in the packet
  int low = response[4];                         //low byte for value is 5th byte in the packet
  long val = (high*256) + low;                //Combine high byte and low byte with this formula to get value
  return val * valMultiplier;
}

// Get voltage ****************************************************************
float voltage(float _adc, int _gain)
{
  float V;
  switch(_gain)
  {
    case 0:  // default 2/3x gain setting for +/- 6.144 V
      V = _adc * 0.0001875;
      break;
    case 1:  // 1x gain setting for +/- 4.096 V
      V = _adc * 0.000125;
      break;
    case 2:  // 2x gain setting for +/- 2.048 V
      V = _adc * 0.0000625;
      break;
    case 3:  // 4x gain setting for +/- 1.024 V
      V = _adc * 0.00003125;
      break;
    case 4:  // 8x gain setting for +/- 0.512 V
      V = _adc * 0.000015625;
      break;
    case 5:  // 16x gain setting for +/- 0.256 V
      V = _adc * 0.0000078125;
      break;

    default:
      V = 0.0;
  }
  return V;
}

// Fill the dots one after the other with a color***********************************************
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// bruh
