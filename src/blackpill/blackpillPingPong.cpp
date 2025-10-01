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
    float center_freq = 920.400000f;  // MHz
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

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!

void setFlag(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}

void setup() {
  Serial.begin(115200);

  delay(5000);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));

  loraSetup();

  // set the function that will be called
  // when new packet is received
  lora.setDio1Action(setFlag);

  // #if defined(INITIATING_NODE)
  //   // send the first packet on this node
  //   Serial.print(F("[SX1262] Sending first packet ... "));
  //   transmissionState = lora.startTransmit("Hello World!");
  //   transmitFlag = true;
  // #else
    // start listening for LoRa packets on this node
    Serial.print(F("[SX1262] Starting to listen ... "));
    int state = lora.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) { delay(10); }
    }
  // #endif
}

void loop() {
  // check if the previous operation finished
  if(operationDone) {
    // reset flag
    operationDone = false;

    if(transmitFlag) {
      // the previous operation was transmission, listen for response
      // print the result
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        Serial.println(F("transmission finished!"));

      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);

      }

      // listen for response
      lora.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data and send another packet
      String str;
      int state = lora.readData(str);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1262] Received packet!"));

        // print data of the packet
        Serial.print(F("[SX1262] Data:\t\t"));
        Serial.println(str);

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[SX1262] RSSI:\t\t"));
        Serial.print(lora.getRSSI());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[SX1262] SNR:\t\t"));
        Serial.print(lora.getSNR());
        Serial.println(F(" dB"));

      }

      // wait a second before transmitting again
      delay(1000);

      // send another one
      Serial.print(F("[SX1262] Sending another packet ... "));
      transmissionState = lora.startTransmit("Hello World!");
      transmitFlag = true;
    }
  
  }
}
