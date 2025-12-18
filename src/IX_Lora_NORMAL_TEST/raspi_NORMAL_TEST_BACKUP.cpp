#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

#include <STM32FreeRTOS.h>

#include <vector>
#include <queue>
#include <utility>

#include "pinoutSX1262.h"
#include "config.h"
#include "IX_state.h"

#define APOGEENOW 1

//               MISO     MOSI      SCLK
SPIClass spi1(LORA_MOSI,LORA_MISO,LORA_SCLK);  

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0);

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// RADIO
bool inTx;
uint32_t tx_cycle; // micros
uint32_t last_finish_tx;
bool enableRadio = false;

// flag
bool txFlag = false;

// UART               RX   TX
HardwareSerial raspi(PA3, PA2);

// PACKET TEMP VARIABLE
int n,i;

// PACKET CONFIG
uint8_t maxPacket = 245;

// COUNTING
uint8_t frameCount;
uint8_t maxFrame = 9;

// PACKET STORAGE
String buffer;
String header,ender;

String chunk;
std::queue<String>packet;

std::vector<byte> top_packet;

uint8_t packet_left = 255;

// INTERVAL
uint32_t raspiCheckInterval = 10000; // millis()
uint32_t lastRaspiCheck;

// STATE
uint8_t stm32_state;

int state;

// TX RASPI
uint32_t tx_apogee_cycle = 1000; // millis
uint32_t last_tx_apogee;

/*
NORMAL : SEND GARBARGE DATA
APOGEE : TRY TO GET IMAGE FROM RASPI
SUCCESS : FINISH GE IMAGE FROM RASPI
*/


extern void intervalTask(void *);

extern void raspiTask(void *);

extern void radioTask(void *);

extern void handle_apogee();

extern void apogee_check(void *);

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);

extern void logInfo(void *);

extern void clear_packet();

void setup() {
  Serial.begin(115200);
  while(!Serial);
  raspi.begin(115200);
  Serial.print(F("[SX1262] Initializing ... "));

  spi1.begin();
  
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

  state = min(state,radio.autoLDRO());
  Serial.print("AutoLDRO: ");
  Serial.println(state);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
    enableRadio = 1;
  } else {
    Serial.print(F("failed, code ")); 
    Serial.println(state);
    delay(2000); 
    enableRadio = 0;
  }

  if(enableRadio){
    radio.setDio1Action(setFlag);
    last_finish_tx = micros();
    tx_cycle = 0;
  }

  // SET INTERVAL
  lastRaspiCheck = millis();
  last_tx_apogee = millis();
  
  stm32_state = NORMAL;

  frameCount = 0;

  Serial.println("Start loop");
  chunk.reserve(maxPacket);

  inTx = false;

  xTaskCreate(intervalTask, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(raspiTask, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(apogee_check, "", 2048, nullptr, 2, nullptr);
  if(enableRadio)
    xTaskCreate(radioTask, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(logInfo, "", 2048, nullptr, 2, nullptr);
  vTaskStartScheduler();
}

void intervalTask(void *){
  while(1){
    if(millis() - lastRaspiCheck > raspiCheckInterval){
      lastRaspiCheck = millis();
      Serial.println("raspi connect: " + String(raspi) + " enable radio: " + String(enableRadio));
      Serial.println("In queue: " + String(packet.size()));
    }
  }
}

void raspiTask(void *){
  while(1){
    if(raspi){
      while(raspi.available()){
        if (stm32_state == NORMAL && !packet.empty()) continue;
        
        buffer = "";
        buffer = raspi.readStringUntil('\n');
        n = buffer.length();

        header = buffer.substring(0,2);
        ender = buffer.substring(n-3,n);
        Serial.println("Get packet range raspi: " + String(n));
        if ((header != "IX" && stm32_state == NORMAL ) || 
            (header != "AP" && stm32_state == APOGEE )){
            Serial.println("DENEID PACKET: UNEXPECT HEADER " + header);
            break;
          }
        if (ender != "END"){
          Serial.println("DENEID PACKET: UNEXPECT ENDER " + ender);
          break;
        }
        
        i = 0;
        
        // printBuffer();
        clear_packet();

        while(n > 0){
          n -= maxPacket;
          chunk = "";

          chunk += ('0' + frameCount);

          chunk += buffer.substring(i, min(i + maxPacket, buffer.length()));

          i += maxPacket;
          chunk += char(max(0,ceil(float(n)/maxPacket)));
          
          packet.push(chunk);
          Serial.println("PACKET LEFT TO SEPERATE: " + String(max(0,ceil(float(n)/maxPacket))));
          Serial.println("CHAR LEFT TO SEPERATE: " + String(n));
          Serial.println();
        }

        frameCount += 1;
        if(frameCount > maxFrame) frameCount = 0;

        if (stm32_state == APOGEE) stm32_state == SUCCESS;
      }


      if (millis() - tx_apogee_cycle > last_tx_apogee){
        last_tx_apogee = millis();
        if (stm32_state == NORMAL && packet.empty()){
          raspi.println("PACKET_PLEASE");
        }
        if (stm32_state == APOGEE){
          raspi.println("CMD_APOGEE");
        } 
      }
    }
  }
}

void radioTask(void *){
  while(1){
    if(txFlag){
      txFlag = false;
      inTx = false;
      if (stm32_state != NORMAL){
        packet.push(std::move(packet.front()));
      }
      packet.pop();
      DELAY(1);
    }
    
    if(!inTx) {
      if(!packet.empty()){
        inTx = true;

        packet_left = int((top_packet)[top_packet.size() - 1]);
        
        state = radio.startTransmit(packet.front());
        
        if (state == RADIOLIB_ERR_NONE) {
          Serial.println(F("[SX1262] Send packet!"));
        } else {
          Serial.print(F("failed, code "));
          Serial.println(state);
        }
      }
    }
  }
}

void handle_apogee(){
    stm32_state = APOGEE;

    clear_packet();
    // packet.clear()    

}

void apogee_check(void *){
    // SHOGUN.EXE
    while(1){
      if(APOGEENOW){
        handle_apogee();
        break;
      }
    }
}


void printBuffer(){
    for(int i = 0;i < n;i++){
      Serial.print((char)(buffer[i]));
    }
    Serial.println();
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

void logInfo(void *){
  while(1){
    Serial.println("===============STATE===============");
    // Serial.println("STATE RADIO: " + String(radio_state));
    Serial.println("STATE RASPI: " + String(stm32_state));
    // Serial.println("===============CYCLE===============");
    // Serial.println("RATIO: " + String(ratio));
    // Serial.println("RATIO STEP: " + String(ratio_step));
    // Serial.println("RATIO MIN: " + String(min_ratio));
    // Serial.println("RATIO CYCLE: " + String(ratio_cycle));
    // Serial.println("RATIO CYCLE MAX: " + String(max_ratio_cycle));
    // Serial.println("TX CYCLE: " + String(tx_cycle));
    // Serial.println("================CMD================");
    // Serial.println("COUNTING CMD: " + String(counting_cmd));
    // Serial.println("OLD CMD FROM GROUND: " + String(old_cmd));
    Serial.println("=============PACKETLEFT============");
    Serial.println("PACKET LEFT FROM QUEUE: " + String(packet.size()));
    Serial.println("PACKET LEFT FROM COUNTING:" + String(packet_left));
    Serial.println("===============RADIO===============");
    Serial.println("IN TX: " + String(inTx));
    // Serial.println("================ACK================");
    // Serial.println("ACK: " + String(ack));
    // Serial.println("NACK: " + String(nack));
    Serial.println();
    DELAY(2000);
  }
}

void clear_packet(){
  while (!packet.empty()) packet.pop();
}

void loop(){}

void printState(){

}