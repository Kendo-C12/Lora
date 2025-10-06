#include <SPI.h>
#include <RadioLib.h>
#include <EEPROM.h>


// SX1276 pin connections for TTGO LoRa32 V1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define DIO1 13

constexpr struct {
    float center_freq = 920.400000f;  // MHz
    float bandwidth   = 125.f;     // kHz
    uint8_t spreading_factor = 9;  
    uint8_t coding_rate = 8;       
    uint8_t sync_word = 0x12;      
    int8_t power = 20;             
    uint16_t preamble_length = 16;
} lora_params;

// Initialize SX1276 module
SX1276 lora = new Module(SS, DIO0, RST, DIO0);

int16_t state;

uint32_t rxTime;

String tx_data;
uint32_t serialEndTime;

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
  rxTime = millis();
}

// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}


void serialReadTask() {
  if (Serial.available())
  {
    tx_data = "";
    serialEndTime = millis();
    while(millis() - serialEndTime < 10){
        while (Serial.available()){
            tx_data += static_cast<char>(Serial.read());
            serialEndTime = millis();
        }
    }
    if (tx_data.substring(0, 4) == "freq")
    {
      String freqStr = tx_data.substring(4);
      freqStr.trim();
      const float freq = freqStr.toFloat();
      lora.setFrequency(freq);
      EEPROM.put<float>(0, freq);
      Serial.print("SetFrequencyTo ");
      Serial.print(freq);
      Serial.println("MHz");
      lora.startReceive();
    }
    // Serial.print("Get Serial");
  }
}

void setup() {
  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));

  loraSetup();

  // set the function that will be called
  // when new packet is received
  lora.setDio1Action(setFlag,RISING);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = lora.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.scanChannel();
}

void loop() {
  serialReadTask();
  // check if the flag is set
  if(receivedFlag) {
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = lora.readData(str);

    delay(1);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      // Serial.println(F("[SX1278] Received packet!"));

      // // print data of the packet
      // Serial.print(F("[SX1278] Data:\t\t"));
      Serial.println(str);

      // // print RSSI (Received Signal Strength Indicator)
      // Serial.print(F("[SX1278] RSSI:\t\t"));
      // Serial.print(lora.getRSSI());
      // Serial.println(F(" dBm"));

      // // print SNR (Signal-to-Noise Ratio)
      // Serial.print(F("[SX1278] SNR:\t\t"));
      // Serial.print(lora.getSNR());
      // Serial.println(F(" dB"));

      // // print frequency error
      // Serial.print(F("[SX1278] Frequency error:\t"));
      // Serial.print(lora.getFrequencyError());
      // Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(state);

    }
    lora.startReceive();
    // rxTime = millis();
  }
}
