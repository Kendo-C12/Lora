#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// BAROMETER
Adafruit_MS8607 ms8607;

// GNSS
SFE_UBLOX_GNSS max10s;

HardwareSerial raspi(PA3, PA2);

const int MAXBYTE = 1024;
byte byteArr[1024];
uint32_t byteCount = 0;
uint32_t lastRxTime = 0;
bool receiving = false;

void setup() {
  Serial.begin(115200);      // USB debug
  raspi.begin(115200);     // Hardware UART

  Wire.begin();

  if (ms8607.begin() == false) {
    while(1){
      Serial.println("MS8607 failed to start");
      delay(2000);
    }
  }
  else{
    Serial.println("MS8607 success to start");
  }
  // MAX10S GNSS
  if (max10s.begin() == false) {
    while(1){
      Serial.println("Max-m10s failed to start");
      delay(2000);
    }
  }
  else{
    Serial.println("Max-m10s success to start");
  }


  Serial.println("STM32 UART RX packet size test");
}

void loop() {
  while (raspi.available()) {
    int n = raspi.readBytes(byteArr,MAXBYTE);
    Serial.println(n);
  }
  if(millis() - 5000 > lastRxTime && millis() > 5000){
    lastRxTime = millis();
    raspi.println("PACKET_PLEASE");
  }
}
