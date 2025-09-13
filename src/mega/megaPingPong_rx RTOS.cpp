#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h> 

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

SPIClass spi1;  // Using hardware SPI (pins 50,51,52)

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0); // 8 MHz for Mega2560

constexpr struct {
    float center_freq = 920.600000f;  // MHz
    float bandwidth   = 125.f;     // kHz
    uint8_t spreading_factor = 9;  
    uint8_t coding_rate = 8;       
    uint8_t sync_word = 0x12;      
    int8_t power = 22;             
    uint16_t preamble_length = 16;
} lora_params;

#define LORA_NSS   53
#define LORA_DIO1  23
#define LORA_NRST  22
#define LORA_BUSY  24

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

void loraSetup(){
    // LoRa
    Serial.println("Initializing SX1262...");

    int16_t lora_state = lora.begin(lora_params.center_freq,
                                    lora_params.bandwidth,
                                    lora_params.spreading_factor,
                                    lora_params.coding_rate,
                                    lora_params.sync_word,
                                    lora_params.power,
                                    lora_params.preamble_length
                                    );
    if(lora_state != RADIOLIB_ERR_NONE) {
        Serial.print("Begin failed, code: ");
        Serial.println(lora_state);
        while(true);
    }

    Serial.print("Lora Begin: ");
    Serial.println(lora_state);

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

    DELAY(5000);
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
float lora_rssi;


void serialReadTask(void *pvParameters);
void rx(void *pvParameters);


void setup()
{
  Serial.begin(115200);

  Serial.println("Connected");

  spi1.begin(); // initialize SPI bus

  Serial.println("SPI begin");

  DELAY(1000);

  Serial.print("Lora setup");

  loraSetup();

  Serial.print("success");

  xTaskCreate(serialReadTask,"Serial Reader",4096,NULL,1,NULL );
  xTaskCreate(rx,"RX",4096,NULL,1,NULL );
  vTaskStartScheduler();
}

void loop(){

}

void serialReadTask(void *pvParameters __attribute__((unused))) {
  while (true) {
    if (Serial.available() > 0) {
        tx_data = static_cast<char>(Serial.read());
        Serial.print("You typed: ");
        Serial.println(tx_data);

        lora_state = LoRaState::TRANSMITTING;
        lora.startTransmit(tx_data);
        lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
        Serial.println("[TRANSMITTING...]");
    }

    DELAY(10);
  }
}

void rx(void *pvParameters __attribute__((unused)))
{
    while(true){
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
}