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
bool operation = false;
bool txflag = false;

// UART               RX   TX
HardwareSerial raspi(PA3, PA2);

// PACKET TEMP VARIABLE
int n,i;

// PACKET CONFIG
uint8_t maxPacket = 245;

// COUNTING
uint8_t frameCount;
uint8_t maxFrame = 12;

// PACKET STORAGE
std::vector<byte> buffer(16384);
std::vector<byte> chunk;
std::queue<std::pair<std::vector<byte>,int>>packet;

std::vector<byte> top_packet;

uint8_t packet_left = 255;

// INTERVAL
uint32_t raspiCheckInterval = 10000; // millis()
uint32_t lastRaspiCheck;

// STATE
uint8_t radio_state;
uint8_t old_radio_state;

uint8_t raspi_state;

uint32_t ack = 0;
uint32_t nack = 0;

int state;
bool haveTx;
bool changeState = false;

// DUTYCYCLE
uint8_t ratio_cycle = 0;
uint8_t max_ratio_cycle = 5;
uint8_t min_ratio = 15;
uint8_t ratio_step = 10;

uint8_t ratio = 15;

// CMD
char counting_cmd = 'A';
char old_cmd = '\0';

// TX RASPI
uint32_t tx_apogee_cycle = 300; // millis
uint32_t last_tx_apogee;

/*
TX 

RADIO STATE
NORMAL - BEFORE APOGEE
    SEND IX FOR TRACKING ✓
    WAIT UNTIL TRICKER BY SHOGUN.EXE THEN ✓
      STATE (RADIO) APOGEE ✓
APOGEE - GET TO APOGEE ; RX NORMAL -> SYNC
    SEND CMD APOGEE ✓
    WAIT FOR CMD SYNC THEN ✓
        STATE (RADIO) SYNC ACKNOWLEGE ✓
    ; DUTYCYCLE 95 (-10) ✓
{
SYNC ACKNOWLEGE - ACKNOWLEGE THAT FROM PASSENGER ; RX SYNC -> ACKNOWLEGE
    SEND CMD SYNC ACKNOWLEGE ✓
    WAIT FOR CMD ACKNOWLEGE THEN ✓
        STATE (RADIO) APOGEE SENDER ✓
    ; DUTYCYCLE 55 (-10) ✓
APOGEE SENDER - SEND IMAGE APOGEE ; RX RECEIVING -> SYNC
    WAIT UNTIL STATE (RASPI) SUCCESS ✓
    START SEND IMAGE APOGEE THEN ✓
      WAIT 5 SEC FOR CMD SYNC ✓
    ; NO DUTYCYCLE USE THAT FIXING ALGO ✓
}LOOP UNTIL PACKET_LEFT -> 0 ✓

LANDING - NO COMMUNICATE ✓

RASPI STATE
NORMAL - GET AND SEND DUMMPY OF SOME IMAGE ; RX NORMAL -> NORMAL
    WAIT FOR APOGEE TRIGER BY SHOGUN.EXE THEN ✓
      STATE (RASPI) GET FULL IMAGE FROM RASPI ✓
APOGEE (GET FULL IMAGE FROM RASPI) - GET IMAGE FROM RASPI ; RX ACKNOWLEGE -> RECEIVING
    SEND CMD APOGEE\n TO RASPI ✓
    WAIT FOR PACKET THEN ✓
      STATE (RADIO) SUCCESS ✓

SUCCESS - GET IMAGE DONE;
    SEND CMD SUCCESS ✓

STRUCTER DATA (RASPI)
IX LENGTH PACKET

STRUCTER DATA (RADIO) : REAL MAX 255
IXAPO  FRAME_COUNT   PACKET    PACKET_LEFT
5   1                MAX 245   1             byte

INCASE OF NORMAL WE CAN USE TCP IN NORMAL WE CAN DENY THE NORMAL AND MAKE IT ALL TCP
IF NOEMAL SEND IMG TOO THEN
    IX -> IXAPO

EVERY CMD HAVE TO HAVE NUMBER OF CMD EX CMD1 
    THE NUMBER WILL REQUIRE 1 BYTE STORAGE
*/
/*
RASPI

NORMAL - BEFORE APOGEE ; STM32 NORMAL -> APOGEE
    WAIT FOR CMD APOGEE\n THEN
      STATE APOGEE
APOGEE - GET TO APOGEE ; STM32 APOGEE -> SUCCESS
    SEND IMG
      IF CMD SUCCESS THEN
        STATE SUCCESS
      ELSE
        SEND AGAIN EVERY 2 SEC
SUCCESS - LANDING ; STM32 SUCCESS

*/

extern void intervalTask(void *);

extern void raspiTask(void *);

extern void radioTask(void *);

extern void handle_radio_command(String cmd);

extern void handle_apogee();

extern void apogee_check(void *);

extern void printBuffer();

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);

extern uint32_t calculate_tx_cycle(uint32_t ToA,uint8_t ratio);

extern void logInfo(void *);

extern void trackState(void *);

extern void handle_counting_cmd();

extern void clear_packet();

void setup() {
  Serial.begin(115200);
  while(!Serial);
  raspi.begin(921600);
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
  

  frameCount = 0;

  Serial.println("Start loop");
  chunk.reserve(maxPacket);

  inTx = false;

  radio_state = NORMAL;
  old_radio_state = -1;

  xTaskCreate(intervalTask, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(raspiTask, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(apogee_check, "", 2048, nullptr, 2, nullptr);
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
        if (raspi_state == SUCCESS){
          break;
        }
        
        clear_packet();

        n = raspi.readBytes(buffer.data(), buffer.size());
        // Serial.println("Get packet range raspi: " + String(n));
        if (char(buffer[0]) != 'I' || char(buffer[1]) != 'X')
          break;
        if (n - 3 != int(buffer[2]))
          break;

        if (raspi_state == APOGEE){
          raspi_state = SUCCESS;
          raspi.println("CMD SUCCESS");
        }
        else if (raspi_state == SUCCESS){
          continue;
        }

        n = int(buffer[2]);
        
        i = 0;
        frameCount += 1;
        if(frameCount > maxFrame) frameCount = 0;

        // printBuffer();
        while(n > 0){
          n -= maxPacket;
          chunk.clear();
          chunk.push_back(byte('I'));
          chunk.push_back(byte('X'));
          
          if(radio_state == APOGEE_SENDER){
            chunk.push_back(byte('A'));
            chunk.push_back(byte('P'));
            chunk.push_back(byte('O'));
          }
          if(radio_state == NORMAL_SENDER){
            chunk.push_back(byte('N'));
            chunk.push_back(byte('O'));
            chunk.push_back(byte('R'));
          }
          chunk.push_back(byte(frameCount));

          chunk.insert(
              chunk.end(),
              buffer.begin() + i,
              min(buffer.begin() + i + maxPacket, buffer.end())
          );
          i += maxPacket;

          // modify
          chunk.push_back(byte(max(0,ceil(n/maxPacket))));
          chunk.push_back(byte(counting_cmd));
          handle_counting_cmd();
          packet.push(std::make_pair(chunk,chunk.size()));
        }

        raspi_state == SUCCESS;
      }
      if (raspi_state == APOGEE && millis() - tx_apogee_cycle > last_tx_apogee){
        last_tx_apogee = millis();
        raspi.println("CMD APOGEE" + counting_cmd);
        handle_counting_cmd();
      }
    }
  }
}

void radioTask(void *){
  while(1){
    if(operation)
    {
      operation = false;
      if(txflag){
        txflag = false;
        inTx = false;
        // Serial.println("Real Time on Air in micro sec: " + String(micros() - last_finish_tx));
        last_finish_tx = micros();
        tx_cycle = calculate_tx_cycle(micros() - last_finish_tx,ratio);
        ratio_cycle += 1;
        if(ratio_cycle >= max_ratio_cycle){
          ratio_cycle = 0;
          ratio -= ratio_step;
          if (ratio < min_ratio) ratio = min_ratio;
        }
        radio.startReceive();
      }
      else{
        String s;
        state = radio.readData(s);
        state = min(state,radio.finishReceive());

        if(s[s.length() - 1] == old_cmd){
          Serial.print("RECEVEING OLD PACKET");
        }
        
        if (state == RADIOLIB_ERR_NONE) {
          handle_radio_command(s);
        } else {
          Serial.print(F("failed, code "));
          Serial.println(state);
        }
        radio.startReceive();
      }
    }
    
    haveTx = false;
    if(!inTx && ( micros() - last_finish_tx > tx_cycle || changeState)) {
      changeState = false;
      switch(radio_state){
        case NORMAL:
          inTx = true;
          haveTx = true;
          state = radio.startTransmit("IX" + counting_cmd);
          handle_counting_cmd();

            break;
        case APOGEE:
          inTx = true;
          haveTx = true;
          state = radio.startTransmit("IX CMD APOGEE" + counting_cmd);
          handle_counting_cmd();

            break;
        case SYNC_ACKNOWLEGE:
          inTx = true;
          haveTx = true;
          state = radio.startTransmit("IX CMD SYNC ACKNOWLEGE" + counting_cmd);
          handle_counting_cmd();

            break;
        case APOGEE_SENDER: 
          if(raspi_state != SUCCESS) break;

          if(!packet.empty()){
            inTx = true;
            haveTx = true;
            top_packet = packet.front().first;
            packet_left = int((top_packet)[top_packet.size() - 1]);

            top_packet.push_back(byte(counting_cmd));
            handle_counting_cmd();

            state = radio.startTransmit(top_packet.data(),packet.front().second);
          }
            break;
        case NORMAL_SENDER:
          if(!packet.empty()){
            inTx = true;
            haveTx = true;
            top_packet = packet.front().first;
            packet_left = int((top_packet)[top_packet.size() - 1]);

            top_packet.push_back(byte(counting_cmd));
            handle_counting_cmd();

           state = radio.startTransmit(top_packet.data(),packet.front().second);
          }
            break;
        case LANDING:
          // NOTIHING HERE
            break;
      }

      if (haveTx){
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

void handle_radio_command(String cmd){
    if(cmd[cmd.length() - 1] == old_cmd){
      return;
    }
    else{
      old_cmd = cmd[cmd.length() - 1];
    }

    ack++;

    cmd = cmd.substring(0,cmd.length() - 1);
    if (cmd == "cmd sync")
    {
      if (radio_state == APOGEE_SENDER) { 
        packet.pop(); 
        if(packet_left == 0 && packet.empty()){
          radio_state = LANDING;
          ratio = 0;
          return;
        }
        else if((packet_left > 0 && packet.empty()) || packet_left == 0 && !packet.empty() ){
          Serial.println("SOMETHING WENT WRONG");
          Serial.print("PACKET LEFT: " + String(packet_left) + "PACKET QUEUE SIZE: " + String(packet.size()));
        }
      }
      else if (radio_state == NORMAL_SENDER) {
        packet.pop();
      }
      
      radio_state = SYNC_ACKNOWLEGE;

      ratio = 55;
    }
    else if (cmd == "cmd acknowlege")
    {
      radio_state = APOGEE_SENDER;
        ratio = 0; // delay 5 sec
    }
    else{
      ack--;
      nack++;  
    }
}

void handle_apogee(){
    raspi_state = APOGEE;
    radio_state = APOGEE;

    clear_packet();
    // packet.clear()
    

    ratio = 95;
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
  operation = true;
}

uint32_t calculate_tx_cycle(uint32_t ToA,uint8_t ratio){ // micro
  if (ratio == 0)
    return 1 * 1000 * 1000;
  return uint32_t(((float)ToA / (100.0 - (float)ratio) ) * ratio);
}

void handle_counting_cmd(){
  counting_cmd += 1;
  if(counting_cmd > 'Z')
    counting_cmd = 'A';
}

void logInfo(void *){
  while(1){
    Serial.println("=====================================================");
    Serial.println("\t===============STATE===============");
    Serial.println("\tSTATE RADIO: " + String(radio_state));
    Serial.println("\tSTATE RASPI: " + String(raspi_state));
    Serial.println("\t===============CYCLE===============");
    Serial.println("\tRATIO: " + String(ratio));
    Serial.println("\tRATIO STEP: " + String(ratio_step));
    Serial.println("\tRATIO MIN: " + String(min_ratio));
    Serial.println("\tRATIO CYCLE: " + String(ratio_cycle));
    Serial.println("\tRATIO CYCLE MAX: " + String(max_ratio_cycle));
    Serial.println("\tTX CYCLE: " + String(tx_cycle));
    Serial.println("\t================CMD================");
    Serial.println("\tCOUNTING CMD: " + String(counting_cmd));
    Serial.println("\tOLD CMD FROM GROUND: " + String(old_cmd));
    Serial.println("\t=============PACKETLEFT============");
    Serial.println("\tPACKET LEFT FROM QUEUE: " + String(packet.size()));
    Serial.println("\tPACKET LEFT FROM COUNTING:" + String(packet_left));
    Serial.println("\t===============RADIO===============");
    Serial.println("\tIN TX: " + String(inTx));
    Serial.println("\t================ACK================");
    Serial.println("\tACK: " + String(ack));
    Serial.println("\tNACK: " + String(nack));
    Serial.println("=====================================================");
    DELAY(2000);
  }
}

void trackState(void *){
  while(1){
    if (old_radio_state != radio_state) {
      changeState = true;
      old_radio_state = radio_state;
    }
  }
}

void clear_packet(){
  while (!packet.empty()) packet.pop();
}

void loop(){}
