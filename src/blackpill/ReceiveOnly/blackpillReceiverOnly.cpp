#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

#define LORA_MOSI PA7
#define LORA_MISO PA6
#define LORA_SCLK PA5

SPIClass spi1(LORA_MOSI,LORA_MISO,LORA_SCLK);  // Using hardware SPI (MISO,MOSI,SCLK)

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0); // 8 MHz for Mega2560

constexpr struct {
    float center_freq = 915.000000f;  // MHz
    float bandwidth   = 125.f;     // kHz
    uint8_t spreading_factor = 9;  
    uint8_t coding_rate = 8;       
    uint8_t sync_word = 0x12;      
    int8_t power = 22;             
    uint16_t preamble_length = 16;
} lora_params;

#define LORA_NSS   PA4
#define LORA_DIO1  PB8
#define LORA_NRST  PB7
#define LORA_BUSY  PB6

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

void loraSetup(){
  
  Serial.println("Initializing SX1262...");

  int16_t lora_state = lora.begin(
    lora_params.center_freq,
    lora_params.bandwidth,
    lora_params.spreading_factor,
    lora_params.coding_rate,
    lora_params.sync_word,
    lora_params.power,
    lora_params.preamble_length
  );

  if(lora_state != RADIOLIB_ERR_NONE) {
    Serial.print("Begin failed, code: ");
    
    while(true){
        Serial.println(lora_state);
        delay(1000);
    };
  }

  Serial.print("Lora Begin: ");
  Serial.println(lora_state);

  // configure module safely
  lora_state = lora.explicitHeader();
  Serial.print("ExplicitHeader: ");
  Serial.println(lora_state);


  lora_state = lora.setCRC(true);
  Serial.print("SetCRC: ");
  Serial.println(lora_state);

  lora_state = lora.autoLDRO();
  Serial.print("AutoLDRO: ");
  Serial.println(lora_state);


  if (lora_state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(lora_state);
    while (true) { delay(10); }
  }
}

/* ================================================================================================== */
// LoRa State
enum class LoRaState
{
  IDLE = 0,
  TRANSMITTING,
  RECEIVING
};

String input;
String tx_data;
String arg = "";
String cmd = "";

int status_lora;
volatile bool rx_flag = false;
volatile bool tx_flag = false;
volatile LoRaState lora_state = LoRaState::IDLE;

uint32_t lora_tx_end_time;
uint32_t serialEndTime;

uint32_t count = 0;
uint32_t t = 0;

float lora_rssi;

uint32_t serialInTime;

bool receivedFlag = true;

void serialReadTask();
void rx();

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void setup()
{
  delay(5000);

  Serial.begin(115200);

  Serial.println("Connected");

  spi1.begin(); // initialize SPI bus

  Serial.println("SPI begin");

  delay(1000);

  Serial.print("Lora setup");

  loraSetup();

  Serial.print("success");


  lora.setPacketReceivedAction(setFlag);

  lora.startReceive();
}

void loop() {
  // check if the flag is set
  if(receivedFlag && lora.getPacketLength() > 0) {
    // reset flag
    receivedFlag = false;
    // you can read received data as an Arduino String
    String str;
    int state = lora.readData(str);

    str.trim();          // remove whitespace/newlines
    int value = str.toInt();
    // Serial.println(value);
    if (value - count != 1 && value - count > 0) {
        t++;
    }
    count = value;
    lora.standby();

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1262] Data:\t\t"));
      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1262] RSSI\t1:\t"));
      Serial.print(lora.getRSSI(1));
      Serial.println(F(" dBm"));

      Serial.print(F("[SX1262] RSSI:\t\t"));
      Serial.print(lora.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1262] SNR:\t\t"));
      Serial.print(lora.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1262] Frequency error:\t"));
      Serial.print(lora.getFrequencyError());
      Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);

    }
    Serial.println(t);
    lora.startReceive();
  }
}
