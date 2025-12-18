#include <Arduino.h>
#include <RadioLib.h>

#include "pinoutSX1276.h"
#include "config.h"
#include "IX_state.h"

SX1276 radio = new Module(SS, DIO0, RST, DIO0);

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// RADIO
bool inTx;
uint8_t radio_state;
int state;

// FLAG
bool rxFlag = false;

// INTERVAL
uint32_t waitingInterval = 5000;
uint32_t lastWaiting;
uint32_t lastPrint;

// PACKET 
uint32_t bytesSent = 0;
byte byteArr[255];
int n;

// header(IX) frameCount(1) packet ender packet_left
String header;
int frameCount;
byte packet[255];
String ender;
int packet_left;

int size_packet;

struct unexpect{
  bool header = false;
  bool ender = false;
} unexpect;

extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);

// BYTE FUNCTION
extern int subByte(byte* byteArr,byte* packet,int i,int j);

extern String byteToString(int i,int j);

extern int onebyteToInt(int i);

extern void printByte(byte* byteArr,int len);

// SETUP
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

  lastWaiting = millis();
  lastPrint = millis();

  radio_state = NORMAL;
}

void loop() 
{
  // radioTask  
  if(rxFlag)
  {
    rxFlag = false;
    n = radio.getPacketLength();
    Serial.println("PACKET LENGTH: " + String(n));
    state = radio.readData(byteArr,n);

    // state = min(state,radio.finishReceive());

    header = byteToString(0,1);
    frameCount = onebyteToInt(3);
    size_packet = subByte(packet, byteArr, 4, n-3);
    ender = byteToString(n-2,n-2);
    packet_left = onebyteToInt(n-1);

    unexpect.header = false;
    unexpect.ender = false;
    if (header != "IX" && header != "AP"){ 
      Serial.println("UNEXPECT HEADER: " + header); 
      unexpect.header = true; 
    }
    if (ender != ","){ 
      Serial.println("UNEXPECT ENDER: " + String(ender)); 
      unexpect.ender = true;
    }
    
    if (state == RADIOLIB_ERR_NONE && !unexpect.header && !unexpect.ender) {
      Serial.println("FC," + String(frameCount));  // FRAME COUNT
      Serial.println("PS," + String(size_packet));
      Serial.print(header + ",");                  // IX, AP,
      // printByte(packet,size_packet); 
      Serial.write(packet,size_packet); Serial.println();
      Serial.println("PL," + String(packet_left)); // PACKET LEFT
      Serial.println("RS," + String(radio.getRSSI())); // RSSI
    } else if(state != RADIOLIB_ERR_NONE)  {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
    else{
      Serial.println("UNKOWN HEADER " + header + " UNKOWN ENDER " + ender);
    }

    radio.startReceive();
  }
  
  // intervalTask
  // if(millis() - lastWaiting > waitingInterval){
  //   lastWaiting = millis();
  // }
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

int subByte(byte* byteArr,byte* packet,int i,int j){
  int q;
  for(int k = i;k <= j;k++,q++){
    byteArr[q] = packet[k];
  }
  return (j - i + 1);
}

String byteToString(int i,int j){
  String s = "";
  for(int k = i;k <= j;k++){
    s += char(byteArr[k]);
  }
  return s;
}

int onebyteToInt(int i){
  return int(byteArr[i] - char(0));
}

void printByte(byte* byteArr,int len){
  for(int i = 0;i < len;i++){
    if (isprint(byteArr[i])) {
      Serial.print((char)byteArr[i]);
    } else {
      Serial.print('.');
    }
  }
  Serial.println();
}