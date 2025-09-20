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
enum class LoRaState
{
  IDLE = 0,
  TRANSMITTING,
  RECEIVING
};

String tx_data;

int status_lora;
volatile bool rx_flag = false;
volatile bool tx_flag = false;
volatile LoRaState lora_state = LoRaState::IDLE;

uint32_t lora_tx_end_time;
uint32_t serialEndTime;
uint32_t tx_time;
bool tx_time_flag = false;

float lora_rssi;

uint32_t state;
int c = 0;
int t = 0;

void serialReadTask();
void rx();

void countLost(String s){
  s.trim();          // remove whitespace/newlines
  int value = s.toInt();

  if (value - c != 1 && value - c > 0) {
      t++;
  }
  c = value;
}

String clean(String s) {
  s.trim();          // remove whitespace/newlines
  while (s.length() > 0 && (s[s.length() - 1] == '\n' || s[s.length() - 1] == '\r')) {
    s = s.substring(0, s.length() - 1);
  }
  return s;
}

void transmitting(){
    lora_state = LoRaState::TRANSMITTING;
    lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
    Serial.print("Transmitting: ");
    Serial.println(tx_data);
    state = lora.startTransmit(tx_data);
    if(state == RADIOLIB_ERR_NONE) {
      Serial.println("[TRANSMITTING...]");
    }
    else {
      Serial.print("Transmit failed, code: ");
      Serial.println(state);
    }
}

void setFlag(void) {
  rx_flag = true;
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

  float freq;
  EEPROM.get<float>(0, freq);
  if(freq > 800){
    lora.setFrequency(freq);
    Serial.print("Set frequency to ");
    Serial.print(freq);
    Serial.println("MHz");
  } 

  lora_tx_end_time = millis();
  tx_time = millis();

  lora.setPacketReceivedAction(setFlag);

  lora.startReceive();
}

void loop(){
    rx();
}

void serialReadTask() {
  if (Serial.available() && millis() > lora_tx_end_time)
  {
    tx_data = "";
    
    serialEndTime = millis();

    while(millis() - serialEndTime < 10){
        while (Serial.available()){
            tx_data += static_cast<char>(Serial.read());
            serialEndTime = millis();
        }
    }
    if (tx_data.substring(0, 9) == "cmd freq ")
    {
      String freqStr = tx_data.substring(9);
      freqStr.trim();
      const float freq = freqStr.toFloat();
      lora.setFrequency(freq);
      EEPROM.put<float>(0, freq);
      Serial.print("Set frequency to ");
      Serial.print(freq);
      Serial.println("MHz");
    }
    else
    {
      tx_flag = true;
    }
  }
}

void rx()
{

  serialReadTask();

  if(millis() > tx_time && tx_time_flag && rx_flag){   
    tx_time_flag = false; 
    transmitting();
  }

  if(lora_state != LoRaState::RECEIVING && millis() > lora_tx_end_time){
    lora.standby();
    Serial.println("Start Receive");
    lora_state = LoRaState::RECEIVING;
    lora.startReceive();
  }

  if (rx_flag && lora.getPacketLength() > 0 && lora_state == LoRaState::RECEIVING)
  {
    rx_flag = false;

    String s;
    state = lora.readData(s);

    if(state == RADIOLIB_ERR_NONE) {

      countLost(s);
      s = clean(s);
      
      lora_rssi = lora.getRSSI();

      Serial.print("RSSI: ");
      Serial.println(lora_rssi);

      Serial.println("[RECEIVED]   ");
      
      Serial.println(s);

      Serial.println(t);

      Serial.println("[RECEIVING...]");
    }
    else {
      Serial.print("Receive failed, code: ");
      Serial.println(state);
    }
  
    lora.standby();
    lora_state = LoRaState::IDLE;
    
    if(tx_flag){
      tx_time_flag = true;
      tx_flag = false;
      tx_time = millis() + 300;
    }
  }
}
