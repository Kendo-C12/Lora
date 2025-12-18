#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

#include <STM32FreeRTOS.h>

#include <vector>
#include <queue>
#include <utility>

#include <pin

#define enable_OOK false

#define LORA_MOSI PA7
#define LORA_MISO PA6
#define LORA_SCLK PA5

SPIClass spi1(LORA_MOSI,LORA_MISO,LORA_SCLK);  // Using hardware SPI (MISO,MOSI,SCLK)

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0); // 8 MHz for Mega2560

#define LORA_NSS   PA4
#define LORA_DIO1  PB8
#define LORA_NRST  PB7
#define LORA_BUSY  PB6

constexpr struct
{
    float center_freq = 921.500000f; // MHz
    float bitRate = 100; // kHz or kbps         must be 0.5 to 300 kbps
    float freqDev = bitRate/2; // kHz                 must be lower than 200 kHz
    float RxBw = 234.3; // bitRate + 2*freqDev + 10; // kHz                     must be 2.6 to 250 kHz
    int8_t power = 10; // dB
    uint16_t preamble_length = 16; // byte
    uint8_t currentLimit = 100; // mA           must be 45 to 120 mA in 5 mA steps and 120 to 240 mA in 10 mA steps.
    uint8_t dataShaping = RADIOLIB_SHAPING_0_5; // GFSK BT 0.5
    uint8_t syncword = 0x12;
} params;

bool inTx;
bool txflag = false;
uint32_t tx_cycle;
uint32_t last_finish_tx;

// FSK modulation can be changed to OOK
// NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
//       Also, data shaping changes from Gaussian filter to
//       simple filter with cutoff frequency. Make sure to call
//       setDataShapingOOK() to set the correct shaping!

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

//                    RX   TX
HardwareSerial raspi(PA3, PA2);

int n,i;
uint8_t maxPacket = 250;

bool enableRadio = false;
std::vector<byte> buffer(16384);
uint32_t raspiCheckInterval = 10000;
uint32_t lastRaspiCheck;
byte frameCount;
std::queue<std::pair<std::vector<byte>,int>>packet;

std::vector<byte> chunk;

extern void intervalTask(void *);
extern void raspiTask(void *);
extern void radioTask(void *);

void printByteAsASCII(byte b) {
    // Printable ASCII range is 32–126
    if (b >= 32 && b <= 126) {
        Serial.print((char)b);   // Print as ASCII character
    } else {
        // Show non-printable bytes as hex
        Serial.print("[0x");
        if (b < 16) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print("]");
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

void setFlag(void){
  txflag = true;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  raspi.begin(921600);
  Serial.print(F("[SX1262] Initializing ... "));

  spi1.begin();
  
int state = radio.beginFSK(
    params.center_freq,
    params.bitRate,
    params.freqDev,
    params.RxBw,
    params.power,
    params.preamble_length
  );
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  state = min(state,radio.setCurrentLimit(100.0));
  state = min(state,radio.setDataShaping(RADIOLIB_SHAPING_NONE));
  uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
  state = min(state,radio.setSyncWord(syncWord, 8));
  state = min(state,radio.setCRC(2));

  Serial.println("Init with BW: " + String(params.RxBw) + " FreqDev: " + String(params.freqDev));

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
    radio.setPacketSentAction(setFlag);
    last_finish_tx = micros();
    tx_cycle = 0;
  }

  lastRaspiCheck = millis();
  frameCount = 0;

  Serial.println("Start loop");
  chunk.reserve(maxPacket);

  inTx = false;

  xTaskCreate(intervalTask, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(raspiTask, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(radioTask, "", 1024, nullptr, 2, nullptr);
  vTaskStartScheduler();
}

void loop() {}

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
        n = raspi.readBytes(buffer.data(), buffer.size());
        // Serial.println("Get packet range raspi: " + String(n));
        i = 0;
        frameCount += 1;
        if(frameCount > 12) frameCount = 0;

        // printBuffer();
        while(n > 0){
          n -= maxPacket;
          chunk.clear();
          chunk.push_back(byte(frameCount));

          chunk.insert(
              chunk.end(),
              buffer.begin() + i,
              buffer.begin() + i + maxPacket
          );

          // modify
          chunk.push_back(byte(ceil(n/maxPacket)));
          packet.push(std::make_pair(chunk,chunk.size()));
        }
      }
    }
//   }
// }

// void radioTask(void *){
//   while(1){
    if(enableRadio){
      if(txflag){
        txflag = false;
        inTx = false;
        // Serial.println("Real Time on Air in micro sec: " + String(micros() - last_finish_tx));
        last_finish_tx = micros();
        tx_cycle = (micros() - last_finish_tx) / 100;
      }

      if(!inTx && micros() - last_finish_tx > tx_cycle) {
        if(!packet.empty()){
          delay(1);
          inTx = true;
          int state = radio.startTransmit(packet.front().first.data(),packet.front().second);
          packet.pop();
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
}