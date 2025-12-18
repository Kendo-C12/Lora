#include <Arduino.h>
#include <RadioLib.h>

#include "pinoutSX1276.h"
#include "config.h"
#include "IX_state.h"

SX1276 radio = new Module(SS, DIO0, RST, DIO0);

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

#define TEST_BPS 0

// RADIO
bool inTx;
uint32_t tx_cycle; // micros
uint32_t last_finish_tx;

// FLAG
bool operation = false;
bool txFlag = false;

// INTERVAL
uint32_t waitingInterval = 5000;
uint32_t lastWaiting;
uint32_t lastPrint;

// PACKET 
uint32_t bytesSent = 0;
byte byteArr[255];

uint8_t packet_left = 255;

// STATE
uint8_t radio_state;
uint8_t old_radio_state;

uint32_t ack = 0;
uint32_t nack = 0;

uint32_t start_state_acknowlege = 0;

int state;
bool haveTx;
bool changeState = false;

// DUTYCYCLE
uint8_t ratio_cycle = 0;
uint8_t max_ratio_cycle = 5;
uint8_t min_ratio = 15;
uint8_t ratio_step = 10;

int8_t ratio = 15;

// CMD
char old_cmd = '\0';
char counting_cmd = 'A';

/*
RX

NORMAL - BEFORE APOGEE ; PASSENGER NORMAL -> APOGEE
    RX PACKET NORMALY UNTIL GET CMD APOGEE THEN ✓
        STATE SYNC ✓
{
SYNC - SEND TO PASSENGER TO THEM THAT RX KNOW IT APOGEE ; PASSENGER APOGEE -> SYNC ACKNOWLEGE
 #  SEND CMD SYNC  ✓
    WAIT FOR SYNC ACKNOWLEGE THEN ✓
        STATE ACKNOWLEGE ✓
    ; DUTYCYCLE 90 (-10) ✓
ACKNOWLEGE - SEND TO PASSENGER TO TURN INTO NORMAL ; PASSENGER SYNC ACKNOWLEGE -> APOGEE SENDER
 #  SEND CMD ACKNOWLEGE FOR TOA/2 THEN ✓
        IF PACKET LEFT == 0 ✓
            STATE LANDING ✓
        STATE RECEIVING ✓
RECEIVING - WAIT FOR PACKET ; PASSENGER APOGEE SENDER -> SYNC ACKNOWLEGE 
    GET APOGEE IMAGE IF WEB IS OKAY BY CMD OKAY THEN ✓
        STATE SYNC ✓
    ELSE IF GET SYNC ACKNOWLEGE THEN ✓
        STATE ACKNOWLEGE ✓
}LOOP UNTIL PACKET_LEFT -> 0 
LANDING - LANDING ; 

INCASE OF NORMAL WE CAN USE TCP IN NORMAL WE CAN DENY THE NORMAL AND MAKE IT ALL TCP

EVERY CMD HAVE TO HAVE NUMBER OF CMD EX CMD1 
    THE NUMBER WILL REQUIRE 1 BYTE STORAGE
*/

extern void serialTask(void *);

extern void radioTask(void *);

extern void intervalTask(void *);

extern void handle_radio_command(String cmd);

// extern void printBuffer();

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);

extern uint32_t calculate_tx_cycle(uint32_t ToA,uint8_t ratio);

extern uint32_t maxToA();

extern void handle_counting_cmd();

extern void trackState(void *);

extern void logInfo(void *);

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

  state = min(state,radio.autoLDRO());
  Serial.print("AutoLDRO: ");
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
    while (true) { delay(10); }
  }

  radio.setDio0Action(setFlag, RISING);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  lastWaiting = millis();
  lastPrint = millis();

  radio_state = NORMAL;
  old_radio_state = NORMAL;

  xTaskCreate(radioTask, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(intervalTask, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(logInfo, "", 2048, nullptr, 2, nullptr);
  // void serialTask(void *);
  vTaskStartScheduler();
}

void loop() {}

void serialTask(void *){
  while(1){
    if(Serial){
      while(Serial.available()){
        String packet_serial;
        packet_serial = Serial.readStringUntil('\n');
        if (packet_serial == "CMD OKAY"){
          radio_state = SYNC;
        }
      }
    }
  }
}

void radioTask(void*){  
  while(1){
    if(operation){
      if(txFlag){
        txFlag = false;
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
      else {
        // RX

        String s;
        state = radio.readData(s);

        if (state == RADIOLIB_ERR_NONE) {
          Serial.print(F("PACKET,"));
          Serial.println(s);

          Serial.print(F("RSSI,"));
          Serial.println(radio.getRSSI());

          Serial.print(F("SNR,"));
          Serial.println(radio.getSNR());

          Serial.print(F("FREQERROR,"));
          Serial.println(radio.getFrequencyError());

          if(s[s.length() - 1] == old_cmd) Serial.print("RECEVEING OLD PACKET");
          if (radio_state != RECEIVING) handle_radio_command(s);

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
          Serial.println(F("CRCERROR"));
        } else {
          Serial.print(F("FAILEDCODE,"));
          Serial.println(state);
        }
        delay(1);
        radio.startReceive();
      }
    }

    haveTx = false;
    if(!inTx && ( micros() - last_finish_tx > tx_cycle || changeState) && radio_state != NORMAL ) {
      changeState = false;
      switch(radio_state){
        case NORMAL:
            break;
        case SYNC:
          inTx = true;
          haveTx = true;
          state = radio.startTransmit("IX CMD SYNC ACKNOWLEGE" + counting_cmd);
            break;
        case ACKNOWLEGE:
          inTx = true;
          haveTx = true;
          state = radio.startTransmit("IX CMD SYNC ACKNOWLEGE" + counting_cmd);
          if (start_state_acknowlege == 0) start_state_acknowlege = millis();

          if (start_state_acknowlege - millis() > maxToA()/2){
            if (packet_left == 0) { radio_state = LANDING; }
            else { radio_state =  RECEIVING; }
          }
            break;
        case RECEIVING:
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

void intervalTask(void *){
  while(1){
    if(millis() - lastWaiting > waitingInterval){
      lastWaiting = millis();
      Serial.println("Wait for packet");
    }

    if (TEST_BPS && millis() - lastPrint >= 1000) {
      lastPrint = millis();
      float kbps = (bytesSent * 8.0) / 1000.0; // Bits per second → kbps
      Serial.print("Throughput: ");
      Serial.print(kbps);
      Serial.println(" kbps");
      bytesSent = 0;
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
    if (cmd == "CMD APOGEE")
    {
      radio_state = SYNC;
    }
    else if (cmd == "CMD SYNC ACKNOWLEGE")
    {
      radio_state = ACKNOWLEGE;
    }
    else if (cmd.length() >= 5 && (cmd.substring(0,5) == "IXAPO" || cmd.substring(0,5) == "IXNOR")){
      packet_left = uint8_t(cmd[cmd.length() - 1]);
    }
    else{
      ack--;
      nack++;  
    }
}

// void printBuffer(){
//     for(int i = 0;i < n;i++){
//       Serial.print((char)(buffer[i]));
//     }
//     Serial.println();
// }

int min(int i,int j){
  if(i < j) return i;
  return j;
}

int max(int i,int j){
  if(i > j) return i;
  return j;
}

uint32_t calculate_tx_cycle(uint32_t ToA,uint8_t ratio){ // micro
  if (ratio == 0)
    return uint32_t(((float)ToA / (100.0 - (float)90) ) * 90);
  return uint32_t(((float)ToA / (100.0 - (float)ratio) ) * ratio);
}

void handle_counting_cmd(){
  counting_cmd += 1;
  if(counting_cmd > 'Z')
    counting_cmd = 'A';
}

uint32_t maxToA(){
  return radio.getTimeOnAir(245);
}

void setFlag(void){
  operation = true;
}

void trackState(void *){
  while(1){
    if (old_radio_state != radio_state) {
      changeState = true;
      old_radio_state = radio_state;

      // SET RATIO
      switch (radio_state)
      {
      case NORMAL:
          break;
      case SYNC:
        ratio = 90;
          break;
      case ACKNOWLEGE:
        ratio = 0;
          break;
      case RECEIVING:
        ratio = -1;
      default:
        break;
      }
    }
  }
}

void logInfo(void *){
  // NOTTHING HERE
}