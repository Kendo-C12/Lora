#include <Arduino.h>
#include <RadioLib.h>


// SX1276 pin connections for TTGO LoRa32 V1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define DIO1 13

// #define RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK  64

constexpr struct
{
    float center_freq = 921.500000f; // MHz
    float bitRate = 80; // kHz or kbps         must be 0.5 to 300 kbps
    float freqDev = bitRate/2; // kHz                 must be lower than 200 kHz
    float RxBw = 250; // bitRate + 2*freqDev + 10; // kHz                     must be 2.6 to 250 kHz
    int8_t power = 10; // dB
    uint16_t preamble_length = 16; // byte
    uint8_t currentLimit = 100; // mA           must be 45 to 120 mA in 5 mA steps and 120 to 240 mA in 10 mA steps.
    uint8_t dataShaping = RADIOLIB_SHAPING_0_5; // GFSK BT 0.5
    uint8_t syncword = 0x12;
} params;

bool rxFlag = false;
bool enable_OOK = false;
uint32_t waitingInterval = 5000;
uint32_t lastWaiting;
uint32_t lastPrint;
uint32_t bytesSent = 0;
byte byteArr[255];

// FSK modulation can be changed to OOK
// NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
//       Also, data shaping changes from Gaussian filter to
//       simple filter with cutoff frequency. Make sure to call
//       setDataShapingOOK() to set the correct shaping!

SX1276 radio = new Module(SS, DIO0, RST, DIO0);

int min(int i,int j){
  if(i < j) return i;
  return j;
}

void setFlag(void){
  rxFlag = true;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  SPI.begin();
  
  Serial.print(F("[SX1276] Initializing ... "));
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
  } else {
    Serial.print(F("failed, code "));
    while (true) { 
      Serial.println(state);
      delay(2000); 
    }
  }

  Serial.println("Init with BW: " + String(params.RxBw) + " FreqDev: " + String(params.freqDev));


  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to change modulation, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setPacketReceivedAction(setFlag);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  lastWaiting = millis();
  lastPrint = millis();
}

void loop() {
  if(millis() - lastWaiting > waitingInterval){
    lastWaiting = millis();
    Serial.println("Wait for packet");
  }
  
  if(rxFlag) {
    rxFlag = false;
    
    int numBytes = radio.getPacketLength();
    bytesSent += numBytes;
    int state = radio.readData(byteArr, numBytes);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.print(F("PACKET,"));
      Serial.write(byteArr, numBytes);

      Serial.print(F("RSSI,"));
      Serial.println(radio.getRSSI());

      Serial.print(F("SNR,"));
      Serial.println(radio.getSNR());

      Serial.print(F("FREQERROR,"));
      Serial.println(radio.getFrequencyError());

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("CRCERROR"));

    } else {
      Serial.print(F("FAILEDCODE,"));
      Serial.println(state);
    }
    delay(1);
    radio.startReceive();
  }

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    float kbps = (bytesSent * 8.0) / 1000.0; // Bits per second → kbps
    Serial.print("Throughput: ");
    Serial.print(kbps);
    Serial.println(" kbps");
    bytesSent = 0;
  }
}
