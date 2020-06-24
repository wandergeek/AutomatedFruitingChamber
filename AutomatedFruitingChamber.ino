#include <OneWire.h>
#include <SoftwareSerial.h>
#include "Wire.h"
#include <SoftwareSerial.h>
#include <serLCD.h>
#include "Time.h"

int readingInterval = 5000;

//-------------- RTC --------------
#if defined(ARDUINO) && ARDUINO >= 100   // Arduino v1.0 and newer
  #define I2C_WRITE Wire.write
  #define I2C_READ Wire.read
#else                                   // Arduino Prior to v1.0
  #define I2C_WRITE Wire.send
  #define I2C_READ Wire.receive
#endif
#define DS1307_I2C_ADDRESS 0x68  // This is the I2C address
byte zero;
byte second_t, minute_t, hour_t, dayOfWeek, dayOfMonth, month_t, year_t;
char  *Day[] = {"","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
char  *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};

//-------------- LCD Screen --------------
int lcdPin = 4;
serLCD lcd(lcdPin);

//-------------- RH meter --------------
int RHPin = A0;

//-------------- Soil Humidity -----------------
int soilHumiditySensorPin = A1;
int soilHumidityPowerPin = 5;
int maxSensorReading = 1024;
int minSensorReading = 0;

//-------------- Temperature Probe --------------
int tempProbePin = 2; //DS18S20 Signal pin on digital 2
OneWire tempProbe(tempProbePin);  // on digital pin 2



void setup(void) {
  Wire.begin();
  Serial.begin(57600);
  zero=0x00;
  lcd.clear();
  lcd.setBrightness(15);
  /*pinMode(soilHumidityPowerPin,OUTPUT);
  digitalWrite(soilHumidityPowerPin, LOW);*/
  getDateDs1307();
  delay(1000);

}

void loop(void) {

/*
  lcd.selectLine(1);
  lcd.print("hello");
  lcd.selectLine(2);
  lcd.print("world");*/

  float temperature = getTemp();
  int RH = getRH(temperature);

Serial.print(temperature);
Serial.print(":");
Serial.println(RH);

  /*lcd.print("Temperature: " + temperature);
  lcd.print(" Humidity: " + RH);*/





  delay(readingInterval); //slow down output

}

float getSoilHumidity() { //returns mapped soil sensor from 0 - 100
  digitalWrite(soilHumidityPowerPin, HIGH);
  delay(10);
  float sensorReading = analogRead(soilHumiditySensorPin);
  digitalWrite(soilHumidityPowerPin, LOW);
  return map(sensorReading,minSensorReading,maxSensorReading,0,100);
}


float getTemp(){    //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if ( !tempProbe.search(addr)) {
      Serial.println("no more sensors on chain, reset search");
      tempProbe.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  tempProbe.reset();
  tempProbe.select(addr);
  tempProbe.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = tempProbe.reset();
  tempProbe.select(addr);
  tempProbe.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = tempProbe.read();
  }

  tempProbe.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

float getRH(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 5.0;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(RHPin);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment

  return trueRH;
}


//------------------- RTC functions -------------------


byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

void setDateDs1307()
{

   second_t = (byte) ((Serial.read() - 48) * 10 + (Serial.read() - 48)); // Use of (byte) type casting and ascii math to achieve result.
   minute_t = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   hour_t  = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   dayOfWeek = (byte) (Serial.read() - 48);
   dayOfMonth = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   month_t = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   year_t= (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   I2C_WRITE(zero);
   I2C_WRITE(decToBcd(second_t) & 0x7f);    // 0 to bit 7 starts the clock
   I2C_WRITE(decToBcd(minute_t));
   I2C_WRITE(decToBcd(hour_t));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   I2C_WRITE(decToBcd(dayOfWeek));
   I2C_WRITE(decToBcd(dayOfMonth));
   I2C_WRITE(decToBcd(month_t));
   I2C_WRITE(decToBcd(year_t));
   Wire.endTransmission();
}

// Gets the date and time from the ds1307 and prints result
void getDateDs1307()
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  I2C_WRITE(zero);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  second_t     = bcdToDec(I2C_READ() & 0x7f);
  minute_t     = bcdToDec(I2C_READ());
  hour_t       = bcdToDec(I2C_READ() & 0x3f);  // Need to change this if 12 hour am/pm
  dayOfWeek  = bcdToDec(I2C_READ());
  dayOfMonth = bcdToDec(I2C_READ());
  month_t      = bcdToDec(I2C_READ());
  year_t       = bcdToDec(I2C_READ());

  if (hour_t < 10)
    Serial.print("0");
  Serial.print(hour_t, DEC);
  Serial.print(":");
  if (minute_t < 10)
    Serial.print("0");
  Serial.print(minute_t, DEC);
  Serial.print(":");
  if (second_t < 10)
    Serial.print("0");
  Serial.print(second_t, DEC);
  Serial.print("  ");
  Serial.print(Day[dayOfWeek]);
  Serial.print(", ");
  Serial.print(dayOfMonth, DEC);
  Serial.print(" ");
  Serial.print(Mon[month_t]);
  Serial.print(" 20");
  if (year_t < 10)
    Serial.print("0");
  Serial.println(year_t, DEC);
}
