#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <math.h>

Adafruit_MS8607 ms8607;
SFE_UBLOX_GNSS max10s;

bool BaroApogee = false;
bool GPSApogee = false;
bool BaroNearApogee = false;

float altFiltered = 0;    
float alpha = 0.08;
float lastAltBaro = 0;
float highestBaro = 0;
float highestGPS = 0;

float ground_pressure = 0;
int sample_num = 20;

sensors_event_t temp, pressure, humidity;

constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT = 250ul;

TwoWire i2c1(PB9,PB8);

long lastTime = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello");
  i2c1.begin();

  if (ms8607.begin(&Wire) == false) {
    while(1) Serial.println("MS8607 failed to start");
  }

  ground_pressure = 0;
  delay(20);
  for(int i = 0;i < sample_num;i++){
    ms8607.getEvent(&pressure, &temp, &humidity);
    ground_pressure += pressure.pressure;
    delay(20);
  }
  ground_pressure /= sample_num;

  if (max10s.begin(Wire,0x42) == false) {
    while(1) Serial.println("Max-m10s failed to start");
  }
  else{
    max10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    max10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR,UBLOX_CUSTOM_MAX_WAIT);
    max10s.setAutoPVT(true, VAL_LAYER_RAM_BBR,UBLOX_CUSTOM_MAX_WAIT);
    max10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR,UBLOX_CUSTOM_MAX_WAIT);
  }
}

void loop() {

  if (millis() - lastTime > 1000){
    lastTime = millis();

    if (max10s.getFixType()==3){
      float altGPS;

      altGPS = max10s.getAltitudeMSL()/1000.0;
      Serial.print("altGPS: ");
      Serial.println(altGPS);

      if (!GPSApogee){
        if (altGPS> highestGPS) {
          highestGPS = altGPS;
        }
        else if (highestGPS > altGPS + 5) {
          Serial.println("GPSApogee reached, begin falling");
          GPSApogee = true;
        }
      }
    }
    if (max10s.getPVT() == true){
      double latitude = max10s.getLatitude() / 1e7;
      double longtitude = max10s.getLongitude() / 1e7;

      Serial.print(F("Lat: "));
      Serial.print(latitude,7);

      Serial.print(F(" Long: "));
      Serial.print(longtitude,7);
      Serial.print(F(" (degrees * 10^-7)"));

      double altitude = max10s.getAltitudeMSL() / 1000.0;
      Serial.print(F(" Alt: "));
      Serial.print(altitude);
      Serial.print(F(" (m)"));

      Serial.println();
    }
  float altBaro;
  sensors_event_t temp, pressure, humidity;
  ms8607.getEvent(&pressure, &temp, &humidity);
  Serial.print("Pressure: ");Serial.print(pressure.pressure); Serial.println(" hPa");
  altBaro= 44300 * (1 - pow((pressure.pressure / ground_pressure), 1.0 / 5.256)); 

  altFiltered = alpha * altBaro + (1 - alpha) * altFiltered;

  lastAltBaro = altFiltered;

  Serial.print("Baro Alt:");
  Serial.print(altBaro);
  Serial.print(" m");


  if (!BaroNearApogee && altBaro > 24000){
    Serial.println("Near Apogee by Baro");
    BaroNearApogee = true;
  }
  
  if (!BaroApogee){
    if (altFiltered > highestBaro) {
      highestBaro = altFiltered;
    }
    else if (highestBaro > altFiltered + 5) {
      Serial.println("BaroApogee reached, begin falling");
      BaroApogee = true;
    }
  }
  }
}