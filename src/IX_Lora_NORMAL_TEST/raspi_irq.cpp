#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

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

// RADIO
bool inTx;
bool enableRadio = false;

// flag
volatile bool txFlag = false;

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

String top_packet;

std::queue<String>packet;

int packet_left = 255;

// STATE
uint8_t stm32_state;
int state;

// INTERVAL

struct interval
{
  uint32_t get_packet = 1 * 1000;
  uint32_t raspi_check = 10 * 1000; 
  uint32_t log = 30 * 1000;
}interval;

struct last
{
  uint32_t get_packet = millis();
  uint32_t raspi_check = millis(); 
  uint32_t log = millis();
}last;


/*
  NORMAL : SEND GARBARGE DATA
  APOGEE : TRY TO GET IMAGE FROM RASPI
  SUCCESS : FINISH GE IMAGE FROM RASPI
*/

extern void handle_apogee();

extern void apogee_check(void *);

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);

extern void clear_packet();

void setup() {
  Serial.begin(115200);
  raspi.begin(115200);
  while(!Serial);

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

  
  // radio.setDio1Action(setFlag);

  stm32_state = NORMAL;

  frameCount = 0;

  Serial.println("Start loop");

  inTx = false;

}

void loop(){
//   // RASPI RX
//   while(raspi.available()){
//     if (stm32_state == NORMAL && !packet.empty()) continue;
    
//     buffer = "";
//     buffer = raspi.readStringUntil('\n');
//     n = buffer.length();

//     header = buffer.substring(0,2);
//     ender = buffer.substring(n-3,n);
//     Serial.println("Get packet range raspi: " + String(n));
//     if ((header != "IX" && stm32_state == NORMAL ) || 
//         (header != "AP" && stm32_state == APOGEE )){
//         Serial.println("DENEID PACKET: UNEXPECT HEADER " + header);
//         break;
//       }
//     if (ender != "END"){
//       Serial.println("DENEID PACKET: UNEXPECT ENDER " + ender);
//       break;
//     }
    
//     i = 0;
    
//     // printBuffer();
//     clear_packet();

//     while(n > 0){
//       n -= maxPacket;
//       chunk = "";

//       chunk += ('0' + frameCount);

//       chunk += buffer.substring(i, min(i + maxPacket, buffer.length()));

//       i += maxPacket;
//       chunk += char(max(0,ceil(float(n)/maxPacket)) + char(0));
      
//       packet.push(chunk);
//       Serial.println("PACKET LEFT TO SEPERATE: " + String(max(0,ceil(float(n)/maxPacket))));
//       // Serial.println("CHAR LEFT TO SEPERATE: " + String(n));
//       // Serial.println();
//     }

//     frameCount += 1;
//     if(frameCount > maxFrame) frameCount = 0;

//     if (stm32_state == APOGEE) stm32_state == SUCCESS;
//   }

//   // RASPI TX
//   if (millis() - interval.get_packet > last.get_packet){
//     last.get_packet = millis();
//     if (stm32_state == NORMAL && packet.empty()){
//       raspi.println("PACKET_PLEASE");
//     }
//     if (stm32_state == APOGEE){
//       raspi.println("CMD_APOGEE");
//     } 
//   }

  // FLAG
  if(txFlag){
    txFlag = false;
    inTx = false;
    if (stm32_state != NORMAL){
      packet.push(std::move(packet.front()));
    }
    if (!packet.empty()){
      packet.pop();
    }

    Serial.println(F("[SX1262] Finish Send!"));
  }
  
  // RADIO
  if(!inTx) {
    if(!packet.empty()){
      inTx = true;

      top_packet = packet.front();
      // top_packet = "Hello,world";

      packet_left = top_packet[top_packet.length() - 1] - char(0);
      
      state = radio.startTransmit(top_packet);
    
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("[SX1262] Send packet!"));
        Serial.println(("[TOA]: " + String(radio.getTimeOnAir(top_packet.length())/ (1000.0 * 1000.0))));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
      }
    }
  }

  if (millis() - last.log > interval.log){
    last.log = millis();

    // Serial.println();
    // Serial.println("===============STATE===============");
    // Serial.println("STATE RASPI: " + String(stm32_state));
    // Serial.println("=============PACKETLEFT============");
    // Serial.println("PACKET LEFT FROM QUEUE: " + String(packet.size()));
    // Serial.println("PACKET LEFT FROM COUNTING: " + String(packet_left));
    // Serial.println("===============RADIO===============");
    Serial.println("IN TX: " + String(inTx));
    // Serial.println();
  }
  
  // // INTERVAL
  // if(millis() - last.raspi_check > interval.raspi_check){
  //   last.raspi_check = millis();
  //   // Serial.println("raspi connect: " + String(raspi) + " enable radio: " + String(enableRadio));
  //   Serial.println("In queue: " + String(packet.size()));
  // }
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
  Serial.println("INTERRUPT");
}

void clear_packet(){
  while (!packet.empty()) packet.pop();
}