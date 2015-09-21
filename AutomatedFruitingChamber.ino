#include <OneWire.h>
#include <SoftwareSerial.h>

int readingInterval = 1000;

//-------------- RH meter --------------
int RHPin = A0;

//-------------- Soil Humidity -----------------
int soilHumidityPin = A0;
int soilHumThresholdUp = 400;
int soilHumThresholdDown = 250;

//-------------- Temperature Probe --------------
int tempProbePin = 2; //DS18S20 Signal pin on digital 2
OneWire tempProbe(tempProbePin);  // on digital pin 2


void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
  int temperature = getTemp();
  int RH = getRH();
  int soilHumidity = getSoilHumidity();

  Serial.print("Temperature: " + temperature);
  Serial.print(" Humidity: " + RH);
  Serial.println(" Soil Humidity: " + soilHumidity);

  delay(readingInterval); //just here to slow down the output so it is easier to read

}

float getSoilHumidity() {
 return analogRead(soilHumidityPin);
}


float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

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

float getRH(){
  float degreesCelsius = getTemp();
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
