
#define MAXPACKETLENGTH 256
#define MAXPACKET 64
#define MAXBUFFER MAXPACKETLENGTH * MAXPACKET

#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// BAROMETER
Adafruit_MS8607 ms8607;

// GNSS
SFE_UBLOX_GNSS max10s;

HardwareSerial raspi(PA3, PA2);

const int MAXBYTE = 10 * 1000;
byte byteArr[MAXBYTE];
uint32_t byteCount = 0;
uint32_t lastRxTime = 0;
bool receiving = false;

struct interval
{
  uint32_t get_packet = 1 * 1000;
  uint32_t raspi_check = 10 * 1000;
  uint32_t log = 1000 * 1000;
  uint32_t baro = 0.1 * 1000;
  uint32_t gps = 0.1 * 1000; 
  uint32_t cal_apogee = 0.1 * 1000;
}interval;

struct last
{
  uint32_t get_packet = millis();
  uint32_t raspi_check = millis(); 
  uint32_t baro = millis();
  uint32_t gps = millis();
  uint32_t log = millis();
  uint32_t cal_apogee = millis();
}last;


// BARO
bool BaroApogee = false;
bool GPSApogee = false;
bool BaroNearApogee = false;

// FILTER ALTITUDE
float altFiltered = 0;
float alpha = 0.08;
float lastAltBaro = 0;
float climbRate = 0;
float highestBaro = 0;
float highestGPS = 0;

// VALUE
float altBaro;
float altGPS;
sensors_event_t temp, pressure, humidity;    



void setup() {
  Serial.begin(115200);      // USB debug
  raspi.begin(115200);     // Hardware UART

  while(!Serial);

  Wire.begin();

  if (max10s.begin() == false) {
    Serial.println("Max-m10s failed to start");
    delay(1000);
  }

  Serial.println("STM32 UART RX packet size test");
}

void loop() {
  while (raspi.available()) {
    Serial.println("RX BUFFER SIZE: " + String(SERIAL_RX_BUFFER_SIZE));
    int n = raspi.readBytes(byteArr,MAXBYTE);
    Serial.println(n);
    
    for(int i  = 0;i < n;i++){
      Serial.print(char(byteArr[i]));
    }
  }


  if(millis() - 5000 > lastRxTime && millis() > 5000){
    lastRxTime = millis();
    raspi.println("PACKET_PLEASE");
    Serial.println("PACKET_PLEASE");
  }

    // GPS
  if(millis() - last.gps > interval.gps){
    last.gps = millis();

    if (max10s.getFixType()==3){
      altGPS = max10s.getAltitudeMSL()/1000.0;

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

    // lat = max10s.getLatitude();
    // lon = max10s.getLongitude();
    // alt = max10s.getAltitude();
    // SIV = max10s.getSIV();
  }
}
