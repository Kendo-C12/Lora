#include <Arduino.h>
#include <RadioLib.h>

#include "pinoutSX1262.h"
#include "config.h"
#include "IX_state.h"

SPIClass spi1(LORA_MOSI,LORA_MISO,LORA_SCLK);  

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0);

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// RADIO
int state;

// FLAG
bool txFlag = false;
bool inTx = false;

// INTERVAL
uint32_t last_tx;
uint32_t tx_cycle = 100;

// PACKET
byte byteArr[255];
int max_packet = 250;

int j = 0;

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);


void setup() {
  delay(5000);
  Serial.begin(115200);
  spi1.begin();
  
  Serial.println(F("[SX1262] Initializing ... "));
  
  state = radio.begin(
    center_freq,
    bandwidth,
    spreading_factor,
    coding_rate,
    sync_word,
    power,
    preamble_length
  );

  Serial.print("INIT: ");
  Serial.println(state);

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

  radio.setPacketReceivedAction(setFlag);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    while (true) { 
      Serial.println(state);
      delay(2000); 
    }
  }

  packet = "";
  for(int i = 0;i < 255;i++){
    packet += char('A' + (i%26));
  }
  last_tx = millis();
}

void loop() 
{
  // radioTask  
  if(txFlag)
  {
    inTx = false;
    txFlag = false;
    
    last_tx = millis();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("[PACKET SENT SUCCESS]");
    } else  {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }

  }

  if(!inTx && millis() - tx_cycle > last_tx){
    inTx = true;
    packet[0] = char(j + '0');
    radio.startTransmit(packet);
    
    j++;
    if(j > 9) j = 0;
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("[PACKET SENT]");
    } else  {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
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
  txFlag = true;
}