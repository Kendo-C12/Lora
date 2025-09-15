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
    float center_freq = 921.500000f;  // MHz
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
volatile bool transmit = false;
volatile LoRaState lora_state = LoRaState::IDLE;

uint32_t lora_tx_end_time;
uint32_t printV;
uint32_t serialEndTime;

float lora_rssi;

uint32_t serialInTime;

uint32_t reciveTime;
int count = 0;
String oldString = "";

void serialReadTask();
void rx();
String calLostTrack(String in);


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

  // float freq;
  // EEPROM.get<float>(0, freq);
  // if(freq > 800){
  //   lora.setFrequency(freq);
  //   Serial.print("Set frequency to ");
  //   Serial.print(freq);
  //   Serial.println("MHz");
  // }

  reciveTime = millis();

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
      transmit = true;
    }
  }
  printV = millis();
}

void rx()
{
    serialReadTask();

    if (millis() > lora_tx_end_time &&
        lora_state != LoRaState::RECEIVING)
    {
        tx_flag = true;
        lora_state = LoRaState::RECEIVING;
        lora.startReceive();
        Serial.println("[RECEIVING...]");
    }

    // Set Rx Done
    if (lora.getPacketLength() > 0
    &&    (lora.getRSSI() != lora_rssi  || millis() - reciveTime > 2200  ))
    {
        reciveTime = millis();
        rx_flag = true;
        if(lora.getRSSI() == lora_rssi){
          count++;
        }
        
        if (transmit)
        {
            Serial.print("[TRANSMITTED] ");
            Serial.println(tx_data);
            tx_flag = false;
            lora_state = LoRaState::TRANSMITTING;
            lora.startTransmit(tx_data);

            lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
            Serial.println("[TRANSMITTING...]");
            transmit = false;
        }

        lora.startReceive();
        
    }

    // On Receive
    if (rx_flag)
    {
        String s;
        lora.readData(s);
        // if(oldString != s){
          Serial.print(s[s.length() - 1]);
          while (s.length() > 0 && (s[s.length() - 1] == '\n' || s[s.length() - 1] == '\r')) {
            s = s.substring(0, s.length() - 1);
          }
          
          lora_rssi = lora.getRSSI();
          s += ',';
          s += lora_rssi;
          s += ',';
          s += lora.getSNR();
          s += ',';
          s += lora.getPacketLength();
          Serial.println(count);
          // s += ',';
          // s += count;
          // s += ',';
          // s += calLostTrack(s);
          Serial.print("RSSI: ");
          Serial.println(lora_rssi);
          Serial.println("[RECEIVED]    ");
          Serial.println(s);
          // Serial.println(oldString);
          rx_flag = false;

          lora_state = LoRaState::RECEIVING;
          lora.startReceive();
          Serial.println("[RECEIVING...]");
          
          oldString = s;
        // }
            // On Transmit
    }

    // if(millis() - printV > 100){
    //   Serial.print(lora.getRSSI());
    //   Serial.print(lora.getPacketLength());
    //   Serial.println(rx_flag);
    //   printV = millis();
    // }
  }