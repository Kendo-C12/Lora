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


constexpr struct
{
    float center_freq = 921.500000f; // MHz
    float bandwidth = 125.f;         // kHz
    uint8_t spreading_factor = 9;    // SF: 6 to 12
    uint8_t coding_rate = 8;         // CR: 5 to 8
    uint8_t sync_word = 0x12;        // Private SX1262
    int8_t power = 20;               // up to 22 dBm for SX1262
    uint16_t preamble_length = 16;
} params;

// Initialize SX1276 module
SX1276 lora = new Module(SS, DIO0, RST, DIO0);

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
float lora_rssi;

static bool state;

extern void set_rxflag();

void setup()
{
  Serial.begin(115200);
  delay(1000);

  SPI.begin();
  // LoRa
  int lora_state = lora.begin(params.center_freq,
                              params.bandwidth,
                              params.spreading_factor,
                              params.coding_rate,
                              params.sync_word,
                              params.power,
                              params.preamble_length
                            );
  state = state || lora.explicitHeader();
  state = state || lora.setCRC(true);
  state = state || lora.autoLDRO();

  lora.setPacketReceivedAction(set_rxflag);

  Serial.printf("SX1262 LoRa %s\n", state == RADIOLIB_ERR_NONE ? "SUCCESS" : "FAILED");
  if (state != RADIOLIB_ERR_NONE)
    Serial.printf("Initialization failed! Error: %d\n", lora_state);

  float freq;
  EEPROM.get<float>(0, freq);
  lora.setFrequency(freq);
  Serial.print("Set frequency to ");
  Serial.print(freq);
  Serial.println("MHz");
}

void loop()
{
  // Tx On Command
  if (Serial.available())
  {
    tx_data = "";
    while (Serial.available())
      tx_data += static_cast<char>(Serial.read());

    if (tx_data.substring(0, 5) == "freq ")
    {
      String freqStr = tx_data.substring(5);
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
      lora_state = LoRaState::TRANSMITTING;
      lora.startTransmit(tx_data);
      lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
      Serial.println("[TRANSMITTING...]");
    }
  }

  // Set Tx Done
  if (millis() > lora_tx_end_time &&
      lora_state != LoRaState::RECEIVING)
  {
    tx_flag = true;
    lora_state = LoRaState::RECEIVING;
    lora.startReceive();
    Serial.println("[RECEIVING...]");
  }

  // Set Rx Done
  if (lora.getPacketLength() > 0 &&
      lora.getRSSI() != lora_rssi)
  {
    rx_flag = true;
  }

  // On Transmit
  if (tx_flag)
  {
    Serial.print("[TRANSMITTED] ");
    Serial.println(tx_data);
    tx_flag = false;
  }

  // On Receive
  if (rx_flag)
  {
    String s;
    lora.readData(s);
    lora_rssi = lora.getRSSI();
    Serial.print("RSSI: ");
    Serial.println(lora_rssi);
    Serial.print("[RECEIVED]    ");
    Serial.println(s);
    rx_flag = false;

    lora_state = LoRaState::RECEIVING;
    lora.startReceive();
    Serial.println("[RECEIVING...]");
  }
}

void set_rxflag()
{
    rx_flag = true;
}