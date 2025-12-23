
#define MAXPACKETLENGTH 245
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
#define ENABLE_SENSOR 1

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
uint8_t maxPacket = MAXPACKETLENGTH - 10;

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
String tem;

// PACKET
byte* top_packet;
FixedQueue<std::pair<byte*,int>>packet(MAXPACKET);
int packet_left = 0;

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

uint32_t time_apogee;

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
  raspi.begin(38400);
  raspi.setTimeout(50);
  // while(!Serial);
  delay(4000);

  Serial.print("RX BUFFER SIZE: ");
  Serial.println(SERIAL_RX_BUFFER_SIZE);

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
  radio.setDio1Action(setFlag);
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

  time_apogee = millis() + 10 * 1000;
}

void loop(){
  // RASPI RX
  if(raspi.available()){
    // Serial.println("RASPI TASK RX");

    n = raspi.readBytes(buffer,MAXBUFFER);
    if(n > 3000) return;
    byte_receiv = n;
    Serial.println("Receiving success");
  
    header = byteToString(buffer,0,1);
    ender = byteToString(buffer,n-3,n-1);

    Serial.print("PACKET LENGTH: ");
    Serial.println(n);
    handle_command(header);
  }

  // RASPI TX
  if (millis() - interval.get_packet > last.get_packet){
    // Serial.println("RASPI TASK TX");
    last.get_packet = millis();
    if (stm32_state == NORMAL && packet.empty()){
      raspi.println("PACK");
    }
    if (stm32_state == APOGEE){
      raspi.println("APO");
    } 
  }

  // FLAG
  if(txFlag){
    // Serial.println("FLAG TASK");
    txFlag = false;
    inTx = false;
    if (stm32_state != NORMAL && !packet.empty()){
      packet.push(packet.getFront());
      packet.pop();
    }
    if (stm32_state == NORMAL && !packet.empty()){
      packet.pop();
    }
    Serial.println(F("[SX1262] Finish Send!"));
  }
  
  // RADIO
  if(!inTx) {
    if(!packet.empty()){
      // Serial.println("RADIO TASK");
      inTx = true;

      top_packet = packet.getFront().first;
      lenChunk = packet.getFront().second;
      // top_packet = "Hello,world";

      if(lenChunk > 0) {
          packet_left = onebyteToInt(top_packet,lenChunk-1);
          Serial.print("len chunk");
          Serial.println(lenChunk);
          state = radio.startTransmit(top_packet,lenChunk);

          ToA =  radio.getTimeOnAir(lenChunk)/ (1000.0);
          if (state == RADIOLIB_ERR_NONE) {
            Serial.println(F("[SX1262] Send packet!"));
            Serial.print("[TOA]: ");
            Serial.println(radio.getTimeOnAir(lenChunk)/ (1000.0 * 1000.0));
            Serial.print("[PACKET LEFT]: ");
            Serial.println(packet_left);
            Serial.print("PACKET LEFT FROM QUEUE: ");
            Serial.println(packet.size());
            tx_start = millis();
          } else {
            Serial.print(F("failed, code "));
            Serial.println(state);
          }
      } else {
          // Handle invalid packet length (should not happen if logic is correct, but safe to pop)
          packet.pop();
          inTx = false; 
      }
    }
  }

  if (millis() - last.log > interval.log){
    // Serial.println("LOG TASK");
    last.log = millis();
    
    Serial.println();
    // Serial.println("===============STATE===============");
    Serial.print("STATE stm32: ");
    Serial.println(stm32_state);
    // Serial.println("=============PACKETLEFT============");
    Serial.print("PACKET LEFT FROM QUEUE: ");
    Serial.println(packet.size());
    // Serial.println("===============RADIO===============");
    Serial.print("IN TX: ");
    Serial.println(inTx);
    if(ENABLE_SENSOR){
      Serial.println("==============BARO==============");
      Serial.print("TEMPERATUE: ");
      Serial.println(temp.temperature);
      Serial.print("PRESSURE: ");
      Serial.println(pressure.pressure);
      Serial.print("HUMIDITY: ");
      Serial.println(humidity.relative_humidity);
      Serial.print("ALT: ");
      Serial.println(altBaro);
      Serial.println("===============GPS===============");
      Serial.print("LAT: ");
      Serial.println(lat);
      Serial.print("LON: ");
      Serial.println(lon);
      Serial.print("ALT: ");
      Serial.println(alt);
      Serial.print("TIME LEFT TO APOGEE: ");
      Serial.println(int32_t(millis()) - int32_t(time_apogee));
    }
    Serial.println();
  }

  if(ENABLE_SENSOR){
    // Serial.println("SENSOR TASK");
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
    // Serial.println("LED TASK");
    last_error_check = millis();
    digitalToggle(LED_BUILTIN);
  }

  if(millis() > time_apogee && stm32_state == NORMAL){
    handle_apogee();
  }
}

void handle_apogee(){
    Serial.println("APOGEE");
    radio.standby();
    inTx = false;

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

    raspi.print("GPS,");
    raspi.print(lat, 6);
    raspi.print(',');
    raspi.print(lon, 6);
    raspi.print(',');
    raspi.println(alt, 2);

  }
  else if(command == "IX" || command == "AP") // PICTURE
  {
    // if(byte_receiv < 6) return
    if (stm32_state == NORMAL && !packet.empty()) return;
    if (stm32_state == APOGEE && command != "AP") return;

    header = byteToString(buffer,0,1);
    ender = byteToString(buffer,n-3,n-1);
    n = subByte(buffer, buffer, 2, n-4, 0);
    buffer_length = n;
    Serial.print("Get packet range raspi: ");
    Serial.println(n);
    if (ender != "END"){
      Serial.print("ENDER FROM RASPI: ");
    Serial.println(ender);
      return;
    }
    
    i = 0;
    
    clear_packet();

    while(n > 0){
      n -= maxPacket;
      
      lenChunk = 0;
      // HEADER
      lenChunk = stringToByte((command + ","),chunk[current_chunk],lenChunk); 
      // FRAMECOUNT
      lenChunk = intToOneByte(frameCount,chunk[current_chunk],lenChunk);
      // PACKET
      lenChunk = subByte(buffer,chunk[current_chunk],i, min(i + maxPacket - 1, buffer_length-1),lenChunk);
      // ENDER
      lenChunk = stringToByte(",",chunk[current_chunk],lenChunk); 
      // PACKET LEFT
      lenChunk = intToOneByte(max(0,ceil(float(n)/maxPacket)),chunk[current_chunk],lenChunk); 
      
      i += maxPacket;
            
      Serial.print("Chunk header: ");
      tem = byteToString(chunk[current_chunk],0,1);
      Serial.println(tem);
      Serial.println(header);
      
      Serial.print("Chunk Ender: ");
      tem = byteToString(chunk[current_chunk],lenChunk-2,lenChunk-2);
      Serial.println(tem);

      packet.push(std::make_pair(chunk[current_chunk],lenChunk));
      Serial.print("PACKET LEFT TO SEPERATE: ");
      Serial.println(max(0,ceil(float(n)/maxPacket)));  
      Serial.print("LEN CHUNK: ");
      Serial.println(lenChunk);
      current_chunk++;
      if(current_chunk >= MAXPACKET) current_chunk = 0;
    }

    frameCount += 1;
    if(frameCount > maxFrame) frameCount = 0;

    if (stm32_state == APOGEE){
       stm32_state = SUCCESS;
    }
    Serial.println(stm32_state);
  }
  else{
    Serial.println("DENEID PACKET: UNEXPECT HEADER ");
    Serial.println(command);
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