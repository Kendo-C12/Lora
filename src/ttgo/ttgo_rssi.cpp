#include <Arduino.h>
#include <RadioLib.h>

#include "pinoutSX1276.h"
#include "config.h"
#include "IX_state.h"

SX1276 radio = new Module(SS, DIO0, RST, DIO0);

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// RADIO
int state;

// FLAG
bool rxFlag = false;

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);


void setup() {
  Serial.begin(115200);
  while(!Serial);
  SPI.begin();
  
  Serial.print(F("[SX1276] Initializing ... "));
  
  state = radio.begin(
    center_freq,
    bandwidth,
    spreading_factor,
    coding_rate,
    sync_word,
    power,
    preamble_length
  );

  state = min(state,radio.explicitHeader());
  Serial.print("ExplicitHeader: ");
  Serial.println(state);


  state = min(state,radio.setCRC(true));
  Serial.print("SetCRC: ");
  Serial.println(state);

  state = min(state,radio.forceLDRO(true));
  Serial.print("ForceLDRO: ");
  Serial.println(state);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    while (true) { 
      Serial.println(state);
      delay(2000); 
    }
  }

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to change modulation, code "));
    Serial.println(state);
    while (true) { 
      Serial.println(state);
      delay(2000); 
    }
  }

  radio.setPacketReceivedAction(setFlag);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    while (true) { 
      Serial.println(state);
      delay(2000); 
    }
  }
}

void loop() 
{
  // radioTask  
  if(rxFlag)
  {
    rxFlag = false;
    String s;
    state = radio.readData(s);
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("PACKET: "  + s);
      Serial.println("RSSI: " + String(radio.getRSSI()));
      Serial.println("SNR: " + String(radio.getSNR()));
      Serial.println();
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
    radio.startReceive();
  }
}

int min(int i,int j){
  if(i < j) return i;
  return j;
}

int max(int i,int j){
  if(i > j) return i;
  return j;
}

void setFlag(void){
  rxFlag = true;
}