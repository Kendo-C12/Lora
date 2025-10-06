#include <Arduino.h>
#include "Arduino_Extended.h"
#include "lib_xcore"

#include <Wire.h>
#include <SPI.h>

#include "RadioLib.h"

#include "orbit_peripheral_def.h"
#include "orbit_pin_def.h"
#include "orbit_state_def.h"

// Device specific
#define THIS_FILE_PREFIX "NOVA_LOGGER_"
#define THIS_FILE_EXTENSION "CSV"

using time_type = uint32_t;
using smart_delay = vt::smart_delay<time_type>;
using on_off_timer = vt::on_off_timer<time_type>;
using task_type = vt::task_t<vt::smart_delay, time_type>;

template <size_t N>
using dispatcher_type = vt::task_dispatcher<N, vt::smart_delay, time_type>;

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// LoRa

// LoRa State
enum class LoRaState
{
    IDLE = 0,
    TRANSMITTING,
    RECEIVING
};

int status_lora;
volatile bool tx_flag = false;
volatile bool rx_flag = false;

volatile LoRaState lora_state = LoRaState::IDLE;
uint32_t lora_tx_end_time;
float lora_rssi;

SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);

constexpr struct
{
    float center_freq = 925.00'000f; // MHz
    float bandwidth = 125.f;         // kHz
    uint8_t spreading_factor = 9;    // SF: 6 to 12
    uint8_t coding_rate = 8;         // CR: 5 to 8
    uint8_t sync_word = 0x12;        // Private SX1262
    int8_t power = 22;               // up to 22 dBm for SX1262
    uint16_t preamble_length = 16;
} params;

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// DATA
struct Data
{
    // 40 bits
    uint32_t timestamp;
    uint8_t counter;

    nova::state_t ps;
    nova::pyro_state_t pyro_a{};
    nova::pyro_state_t pyro_b{};

    // 160 bits
    double gps_latitude;
    double gps_longitude;
    float gps_altitude;

    // 96 bits
    float altitude;
    float temp;
    float press;

    // 384 bits
    struct
    {
        vec3_u<double> acc;
        vec3_u<double> gyro;
    } imu;

    float currentServo;
    float currentEXT;
    float voltageMon;

    uint8_t last_ack{};
    uint8_t last_nack{};

} data;

// Software control
dispatcher_type<32> dispatcher;

// Communication data
String constructed_data;
String tx_data;
time_type tx_interval = nova::config::TX_IDLE_INTERVAL;
time_type log_interval = nova::config::LOG_IDLE_INTERVAL;

extern void construct_data();

extern void transmit_receive_data();

extern void set_rxflag();

extern void handle_command(String rx_message);

void setup()
{
    Serial.begin(115200);
    delay(2000);

    spi1.begin();

    // LoRa
    int lora_state = lora.begin(params.center_freq,
                                     params.bandwidth,
                                     params.spreading_factor,
                                     params.coding_rate,
                                     params.sync_word,
                                     params.power,
                                     params.preamble_length,
                                     0,
                                     false);
    lora_state = lora_state || lora.explicitHeader();
    lora_state = lora_state || lora.setCRC(true);
    lora_state = lora_state || lora.autoLDRO();

    // lora.setPacketReceivedAction(set_rxflag);

    if (lora_state == RADIOLIB_ERR_NONE)
    {
        Serial.println("SX1262 initialized successfully!");
    }
    else
    {
        Serial.print("Initialization failed! Error: ");
        Serial.println(lora_state);
        while (true)
            ;
    }

    // Tasks
    dispatcher << task_type(transmit_receive_data, 10ul, millis, 252)
            //    << task_type(print_data, 1000ul, millis, 253)
               << task_type(construct_data, 25ul, millis, 254);
            //    << (task_type(save_data, &log_interval, 255), pvalid.sd);

    // Low power mode
    // LowPower.begin();
    dispatcher.reset();
}

void loop()
{
    dispatcher();
}

void construct_data()
{
    constructed_data = "";
    csv_stream_crlf(constructed_data)
        // << data.counter
        // << data.timestamp

        // << nova::state_string(data.ps)
        // << String(data.gps_latitude, 6)
        // << String(data.gps_longitude, 6)
        // << String(data.altitude, 4)
        // << String(ground_truth.apogee, 4)

        // << nova::pyro_state_string(data.pyro_a)
        // << nova::pyro_state_string(data.pyro_b)

        // << data.temp
        // << data.press

        // << data.imu.acc.x << data.imu.acc.y << data.imu.acc.z
        // << data.imu.gyro.x << data.imu.gyro.y << data.imu.gyro.z

        // << data.currentEXT
        // << data.currentServo
        // << data.voltageMon

        // << data.last_ack
        << data.last_nack;

    tx_data = "";
    csv_stream_crlf(tx_data)
        // << "<1>" // DEVICE NO
        // << params.center_freq
        // << data.counter

        // << nova::state_string(data.ps)
        // << String(data.gps_latitude, 6)
        // << String(data.gps_longitude, 6)
        // << String(data.altitude, 4)
        // << String(ground_truth.apogee, 4)

        // << data.currentEXT
        // << data.currentServo
        // << data.voltageMon

        // << data.last_ack
        << data.last_nack;
}

void transmit_receive_data()
{
    static smart_delay nb_trx(2000ul, millis);

    // Tx Loop
    nb_trx([&]() -> void
           {
    lora_state = LoRaState::TRANSMITTING;
    lora.startTransmit(tx_data);
    lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
    Serial.print("[TRANSMITTING...]: "); 
    Serial.println(tx_data);
    ++data.counter; });

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
        Serial.println("[TRANSMITTED] ");
        tx_flag = false;
    }

    // On Receive
    if (rx_flag)
    {
        rx_flag = false;
        String rx_string;
        lora.readData(rx_string);
        lora_rssi = lora.getRSSI();
        Serial.print("[RECEIVED]    ");
        Serial.println(rx_string);
        rx_flag = false;

        Serial.println("RSSI: " + String(lora_rssi));
        handle_command(rx_string);

        lora_state = LoRaState::RECEIVING;
        lora.startReceive();
        Serial.println("[RECEIVING...]");
    }
}

void handle_command(String rx_message)
{
    if (rx_message.substring(0, 4) != "cmd ")
    {
        // Return if cmd header is invalid
        Serial.print("Receive: ");
        Serial.println(rx_message);
        ++data.last_nack;
        return;
    }

    String command = rx_message.substring(4);
    command.trim();

    ++data.last_ack;

    if (command == "ping" || command == "wake" || command == "on")
    {
        // <--- Maybe a wakeup command --->
    }
    else if (command == "arm")
    {

    }
    else if (command == "disarm")
    {

    }
    else if (command == "pad")
    {

    }
    else if (command == "shutup")
    {

    }
    else if (command == "servo-a-set")
    {

    }
    else if (command == "servo-a-lock")
    {

    }
    else if (command == "servo-a-deploy")
    {
 
    }
    else if (command == "servo-b-set")
    {

    }
    else if (command == "servo-b-lock")
    {

    }
    else if (command == "servo-b-deploy")
    {

    }
    else if (command == "launch-override")
    {

    }
    else if (command == "recover")
    {

    }
    else if (command == "zero")
    {

    }
    else if (command == "sleep")
    {

    }
    else if (command == "shutdown")
    {

    }
    else if (command == "reboot" || command == "restart")
    {

    }
    else
    {
        Serial.println(command);
        // <--- Unknown command: send back nack --->
        ++data.last_nack;
        --data.last_ack;
    }
}

void set_rxflag()
{
    rx_flag = true;
}