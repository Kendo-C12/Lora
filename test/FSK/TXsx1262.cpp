#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

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

bool txflag = false;
uint32_t tx_period = 2000;
uint32_t last_tx;

// FSK modulation can be changed to OOK
// NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
//       Also, data shaping changes from Gaussian filter to
//       simple filter with cutoff frequency. Make sure to call
//       setDataShapingOOK() to set the correct shaping!

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

void setFlag(void){
  txflag = true;
}

int min(int i,int j){
  if(i < j) return i;
  return j;
}
void setup() {
  Serial.begin(115200);
  while(!Serial);

  spi1.begin();

  // initialize SX1262 FSK modem with default settings
  Serial.print(F("[SX1262] Initializing ... "));
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
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to set configuration, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }


  radio.setPacketSentAction(setFlag);
  
  last_tx = millis();
}

void loop() {

  if(txflag){
    txflag = false;
    Serial.println("Real Time on Air: " + String(millis() - last_tx));
  }

  if(millis() - last_tx > tx_period) {
    last_tx = millis();
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.startTransmit(byteArr,8);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("[SX1262] Send packet!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
  }
}
