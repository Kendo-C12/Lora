
#define MAXPACKETLENGTH 254
#define MAXPACKET 12
#define MAXBUFFER MAXPACKETLENGTH * MAXPACKET


#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

#include "fixed_queue.h"

#include "pinoutSX1262.h"
#include "config.h"
#include "IX_state.h"

#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// ENABLE MS8607 BAROMETER AND UBLOX GNSS V3
#define ENABLE_SENSOR 0

// Lora
SPIClass spi1(LORA_MOSI,LORA_MISO,LORA_SCLK);  

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0);

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// BAROMETER
Adafruit_MS8607 ms8607;

// GNSS
SFE_UBLOX_GNSS max10s;

// SIZE OF BYTE RECEIVE
uint32_t byte_receiv = 0;

// RADIO
bool inTx;
bool enableRadio = false;
uint32_t ToA;
uint32_t tx_start;

// flag
volatile bool txFlag = false;

// PACKET TEMP VARIABLE
int n,i;

// PACKET CONFIG
uint8_t maxPacket = 245;

// COUNTING
uint8_t frameCount;
uint8_t maxFrame = 9;

// PACKET BUFFER
byte buffer[MAXBUFFER];
int buffer_length;

// UART               RX   TX
HardwareSerial raspi(PA3, PA2);

// HEADER ENDER
String header,ender;

// CHUNK TEMPERARY
byte chunk[MAXPACKET][MAXPACKETLENGTH];
int current_chunk = 0;
int lenChunk = 0;

// PACKET
byte* top_packet;
FixedQueue<std::pair<byte*,int>>packet(MAXPACKET);
int packet_left = 255;

// STATE
uint8_t stm32_state;
int state;

// INTERVAL
struct interval
{
  uint32_t get_packet = 1 * 1000;
  uint32_t raspi_check = 10 * 1000;
  uint32_t log = 10 * 1000;
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

// GPS
std::tuple<float,float,float> gps;
float lat,lon,alt;
byte SIV;

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

uint32_t uart_errors = 0;
uint32_t last_error_check = millis();

// VALUE
/*
  NORMAL : SEND GARBARGE DATA
  APOGEE : TRY TO GET IMAGE FROM RASPI
  SUCCESS : FINISH GE IMAGE FROM RASPI
*/

extern void handle_apogee();

extern void apogee_check(void *);

extern void handle_command(String command);

extern std::tuple<float,float,float> get_gnss();

// SHORT FUNCTION
extern int min(int i,int j);

extern int max(int i,int j);

extern void setFlag(void);

extern void clear_packet();

// BYTE FUNCTION
extern int subByte(byte* packet,byte* byteArr,int i,int j,int len);

extern String byteToString(byte* byteArr,int i,int j);

extern int onebyteToInt(byte* byteArr,int i);

extern int stringToByte(String s,byte* byteArr,int len);

extern int intToOneByte(int i,byte* byteArr,int len);

extern void printByte(byte* byteArr,int len);

// SETUP
void setup() {
  Serial.begin(115200);
  raspi.begin(115200);
  raspi.setTimeout(50);
  // while(!Serial);
  delay(4000);

  Serial.println("RX BUFFER SIZE: " + String(SERIAL_RX_BUFFER_SIZE));

  Serial.println(F("[SX1262] Initializing ... "));

  // SPI
  spi1.begin();
  Wire.begin();

  Wire.setTimeout(10);

  // RADIO
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
  state = min(state,radio.setCRC(true));
  state = min(state,radio.forceLDRO(true));

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
  Serial.println("INIT BARO AND GNSS");

  if(ENABLE_SENSOR){
    // // MS8607 BAROMETER
    if (ms8607.begin() == false) {    
      while(1){
        Serial.println("MS8607 failed to start");
        delay(1000);
      }
    }
    // MAX10S GNSS
    if (max10s.begin() == false) {
      while(1){
        Serial.println("Max-m10s failed to start");
        delay(1000);
      }
    }
  }

  // STATE
  stm32_state = NORMAL;

  // IMAGE
  frameCount = 0;

  // RADIO
  inTx = false;

  // LED CHECKING
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  // CLEAN BUFFER
  while(raspi.available()) { raspi.read(); } 

  // NVIC_SetPriority(USART2_IRQn, 0);  // Highest priority for UART
  // NVIC_SetPriority(EXTI1_IRQn, 2);   // Lower priority for LoRa DIO1

  Serial.println("Start loop");
}

void loop(){
  // RASPI RX
  while(raspi.available()){
    // digitalWrite(LED_BUILTIN,HIGH);
    Serial.println("AVAILABLE: " + String(raspi.available()));
    Serial.println("Start Receiving");
    Serial.println("HAVE RECEIVE: " + String(byte_receiv));
    n = raspi.readBytes(buffer,MAXBUFFER);
    if(n > 3000) continue;
    byte_receiv += n;
    Serial.println("Receiving success");
  
    header = byteToString(buffer,0,1);
    ender = byteToString(buffer,n-3,n-1);

    Serial.println("HEADE FROM RASPI: " + header);
    Serial.println("ENDER FROM RASPI: " + ender);
    Serial.println("PACKET LENGTH: " + String(n));
    handle_command(header);
    // digitalWrite(LED_BUILTIN,LOW);
  }

  // RASPI TX
  if (millis() - interval.get_packet > last.get_packet){
    last.get_packet = millis();
    if (stm32_state == NORMAL && packet.empty()){
      raspi.println("PACKET_PLEASE");
    }
    if (stm32_state == APOGEE){
      raspi.println("CMD_APOGEE");
    } 
  }

  // FLAG
  if(/*txFlag || */(inTx && millis() - tx_start > (ToA * 2) + 10)){
    // if(inTx && millis() - tx_start > ToA * 2) Serial.println("FLAG BY MAXIMUM TOA");
    txFlag = false;
    inTx = false;
    if (stm32_state != NORMAL){
      packet.push(packet.getFront());
      packet.pop();
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

      top_packet = packet.getFront().first;
      lenChunk = packet.getFront().second;
      // top_packet = "Hello,world";

      packet_left = onebyteToInt(top_packet,lenChunk-1);
      
      state = radio.startTransmit(top_packet,lenChunk);

      ToA =  radio.getTimeOnAir(lenChunk)/ (1000.0);
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("[SX1262] Send packet!"));
        Serial.println("[TOA]: " + String(radio.getTimeOnAir(lenChunk)/ (1000.0 * 1000.0)));
        Serial.println("[PACKET LEFT]: " + String(packet_left));
        Serial.println("PACKET LEFT FROM QUEUE: " + String(packet.size()));
        tx_start = millis();
      } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
      }
    }
  }

  if (millis() - last.log > interval.log){
    last.log = millis();
    
    Serial.println();
    Serial.println("===============STATE===============");
    Serial.println("STATE stm32: " + String(stm32_state));
    Serial.println("=============PACKETLEFT============");
    Serial.println("PACKET LEFT FROM QUEUE: " + String(packet.size()));
    // Serial.println("PACKET LEFT FROM COUNTING: " + String(packet_left));
    // Serial.println("NEED PACKET? : " + String(stm32_state == NORMAL && packet.empty()));
    Serial.println("===============RADIO===============");
    Serial.println("IN TX: " + String(inTx));
    Serial.println("==============BARO==============");
    Serial.println("TEMPERATUE: " + String(temp.temperature));
    Serial.println("PRESSURE: " + String(pressure.pressure));
    Serial.println("HUMIDITY: " + String(humidity.relative_humidity));
    Serial.println("ALT: " + String(altBaro));
    Serial.println("===============GPS===============");
    Serial.println("LAT: " + String(lat));
    Serial.println("LON: " + String(lon));
    Serial.println("ALT: " + String(alt));
    Serial.println("SIV: " + String(SIV));
    Serial.println();
  }
  
  // INTERVAL
  if(millis() - last.raspi_check > interval.raspi_check){
    last.raspi_check = millis();
    Serial.println("In queue: " + String(packet.size()));
  }

  if(ENABLE_SENSOR){
    // BAROMETER
    if(millis() - last.baro > interval.baro){
      last.baro = millis();

      ms8607.getEvent(&pressure, &temp, &humidity);
      altBaro = 44300 * (1 - pow((pressure.pressure / 1013.25), 1.0 / 5.256));

      altFiltered = alpha * altBaro + (1 - alpha) * altFiltered;

      climbRate = altFiltered - lastAltBaro;
      lastAltBaro = altFiltered;
    }

    // GPS
    if(millis() - last.gps > interval.gps){
      last.gps = millis();

      if (max10s.getPVT()){
        altGPS = max10s.getAltitudeMSL()/1000.0;

        lat = max10s.getLatitude()/1000.0;
        lon = max10s.getLongitude()/1000.0;
        alt = max10s.getAltitude()/1000.0;

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
    }

    // ALITTUDE
    if(!BaroApogee){
      if(millis() - last.cal_apogee < interval.cal_apogee){
        last.cal_apogee = millis();
        if (altFiltered > highestBaro) {
          highestBaro = altFiltered;
        }
      }
    
      // WARNING
      if (!BaroNearApogee && climbRate < 0.2 && climbRate > -0.2 && altBaro > 1000){
          Serial.println("Near Apogee by Baro");
          BaroNearApogee = true;
      }
    }
  }
  if(millis() - last_error_check > 100){
    last_error_check = millis();
    digitalToggle(LED_BUILTIN);
  }
}

void handle_apogee(){
    stm32_state = APOGEE;

    clear_packet();   
}

void apogee_check(void *){
    // SHOGUN.EXE
    // while(1){
    //   if(APOGEENOW){
    //     handle_apogee();
    //     break;
    //   }
    // }
}

void handle_command(String command){
  if(command.substring(0,2) == "GG") // GET GPS
  {
    Serial.println("RECEIVEING GG");
    gps = get_gnss();
    
    lat = std::get<0>(gps);
    lon = std::get<1>(gps);
    alt = std::get<2>(gps);

    raspi.println("GPS,"
        + String(lat) + ','
        + String(lon) + ','
        + String(alt));


  }
  else if(command == "IX" || command == "AP") // PICTURE
  {
    if (stm32_state == NORMAL && !packet.empty()) return;
    header = byteToString(buffer,0,1);
    ender = byteToString(buffer,n-3,n-1);
    n = subByte(buffer, buffer, 2, n-4, 0);
    buffer_length = n;
    Serial.println("Get packet range raspi: " + String(n));
    if (ender != "END"){
      Serial.println("DENEID PACKET: UNEXPECT ENDER " + ender);
      return;
    }
    
    i = 0;
    
    clear_packet();

    while(n > 0){
      n -= maxPacket;
      
      lenChunk = 0;
      // HEADER
      lenChunk = stringToByte((header + ","),chunk[current_chunk],lenChunk); 
      // FRAMECOUNT
      lenChunk = intToOneByte(frameCount,chunk[current_chunk],lenChunk);
      // PACKET
      lenChunk = subByte(buffer,chunk[current_chunk],i, min(i + maxPacket - 1, buffer_length-1),lenChunk);
      // ENDER
      lenChunk = stringToByte(",",chunk[current_chunk],lenChunk); 
      // PACKET LEFT
      lenChunk = intToOneByte(max(0,ceil(float(n)/maxPacket)),chunk[current_chunk],lenChunk); 
      
      i += maxPacket;
            

      packet.push(std::make_pair(chunk[current_chunk],lenChunk));
      Serial.println("PACKET LEFT TO SEPERATE: " + String(max(0,ceil(float(n)/maxPacket))));  
      // printByte(chunk,lenChunk);
      current_chunk++;
      if(current_chunk > MAXPACKET) current_chunk = 0;
    }

    frameCount += 1;
    if(frameCount > maxFrame) frameCount = 0;

    if (stm32_state == APOGEE) stm32_state = SUCCESS;
  }
  else{
    Serial.println("DENEID PACKET: UNEXPECT HEADER " + command);
    Serial.println((command.substring(0,2) == "GG"));
    Serial.println((command[0] == 'G'));
    Serial.println((command[1] == 'G'));
  }

}

std::tuple<float,float,float> get_gnss(){
  if (ENABLE_SENSOR){
    return std::make_tuple(lat,lon,altBaro);
  }
  else{
    return std::make_tuple(1.0,1.0,1.0);
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

void clear_packet(){
  while (!packet.empty()) packet.pop();
}

int subByte(byte* byteArr,byte* packet,int i,int j,int len){
  for(int k = i;k <= j;k++){
    packet[len] = byteArr[k];
    len++;
  }
  return len;
}

String byteToString(byte* byteArr,int i,int j){
  String s = "";
  for(int k = i;k <= j;k++){
    s += char(byteArr[k]);
  }
  return s;
}

int onebyteToInt(byte* byteArr,int i){
  return int(byteArr[i] - char(0));
}

int stringToByte(String s,byte* byteArr,int len){
  for(int k = 0;k < s.length();k++,len++){
    byteArr[len] = byte(s[k]);
  }
  return len;
}

int intToOneByte(int i,byte* byteArr,int len){
  byteArr[len] = byte(i);
  len++;
  return len;
}

void printByte(byte* byteArr,int len){
  String s = "";
  s = byteToString(byteArr,0,len);
  Serial.println(s);
}