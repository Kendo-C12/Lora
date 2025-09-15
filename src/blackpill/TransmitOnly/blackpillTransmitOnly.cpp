#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>

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
    int8_t power = -9;             
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

// Lora State
enum class LoRaState
{
  IDLE = 0,
  TRANSMITTING,
  RECEIVING
};

int transmissionState = RADIOLIB_ERR_NONE;

bool transmitFlag = false;

volatile bool operationDone = false;

int state;
uint8_t t;
uint8_t last_ack;
uint8_t last_nack;
uint32_t lora_tx_end_time;
volatile LoRaState lora_state = LoRaState::IDLE;
String line;
String stateR = "STARTUP";
float lora_rssi;
volatile bool rx_flag = false;
volatile bool tx_flag = false;
unsigned long last_time;
unsigned long last_time_line;

void setFlag(void) {
  rx_flag = true;
}

void setup() {
  delay(5000);

  last_ack = 0;
  last_nack = 0;

  Serial.begin(115200);
  spi1.begin(); // initialize SPI bus

  delay(1000);

  loraSetup();

  
  //lora.setDio1Action(setFlag);

  t = 0;
  randomSeed(analogRead(A0));

  last_time = millis();
  last_time_line = millis();
}

void loop() {
  if(millis() - last_time_line > 2000){
    line = String(t);

    // line += "<3>";
    // line += ",";
    // line += String(t);
    // line += ",";
    // line += stateR;
    // line += ",";
    // line += String(random(50, 150)); // lat
    // line += ",";
    // line += String(random(50, 150)); // lon
    // line += ",";
    // line += String(random(50, 150)); // alt
    // line += ",";
    // line += String(random(50, 150)); // apogee
    // line += ",";
    // line += String(random(50, 150)); // volMon
    // line += ",";
    // line += String(last_ack);
    // line += ",";
    // line += String(last_nack);

    last_time_line = millis();
  }


  if(millis() - last_time > 2000){

    lora_state = LoRaState::TRANSMITTING;
    lora.startTransmit(line);
    lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(line.length())) / 1000;
    Serial.println("[TRANSMITTING...]"); 
    ++t;

    last_time = millis();
  }
}
