#ifndef NOVA_PERIPHERAL_DEF_KENDO_H
#define NOVA_PERIPHERAL_DEF_KENDO_H

#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

#define SEALEVELPRESSURE_HPA (1013.25)

// nova::config 

constexpr auto HZ_TO_INTERVAL_MS    = [](const double FREQUENCY_HZ) -> uint32_t {
    return static_cast<uint32_t>(1000. / FREQUENCY_HZ);
};
#define INTERVAL_MS_TO_HZ [](const uint32_t INTERVAL) -> uint32_t { return 1000 / INTERVAL; };

#define INA236_ADDRESS = 0x40;

// #define  RANGE_80MV = 0,
// #define  RANGE_20MV = 1

constexpr double TIME_TO_APOGEE_MIN  = 14.69 * 1000;
constexpr double TIME_TO_APOGEE_MAX  = 15.69 * 1000; // Sim 14.69
constexpr double TIME_TO_BURNOUT_MIN = 1.85  * 1000;
constexpr double TIME_TO_BURNOUT_MAX = 2.85  * 1000; // Sim 1.85

// algo
constexpr double LAUNCH_ACC    = 30.0;   // m/s^2
constexpr double APOGEE_VEL    = 5.0;    // m/s
constexpr float  MAIN_ALTITUDE = 150.0f; // m

#define RFD900X_BAUD         = 460800;
#define RPI_BAUD             = 115200;
#define UBLOX_CUSTOM_MAX_WAIT     = 250ul;  // u-blox GPS comm timeout
#define SD_SPI_CLOCK_MHZ          = 20ul;   // 20 MHz
#define MESSAGE_BUFFER_SIZE         = 512ul;

#define TX_IDLE_INTERVAL          = HZ_TO_INTERVAL_MS(1);  // 1 Hz
#define TX_ARMED_INTERVAL         = HZ_TO_INTERVAL_MS(2);  // 2 Hz
#define TX_PAD_PREOP_INTERVAL     = HZ_TO_INTERVAL_MS(4);  // 4 Hz
#define TX_ASCEND_INTERVAL        = HZ_TO_INTERVAL_MS(5);  // 5 Hz
#define TX_DESCEND_INTERVAL       = HZ_TO_INTERVAL_MS(4);  // 4 Hz

constexpr uint32_t LOG_GROUND_INTERVAL         = HZ_TO_INTERVAL_MS(1);   // 1 Hz
constexpr uint32_t LOG_PAD_PREOP_INTERVAL    = HZ_TO_INTERVAL_MS(10);  // 10 Hz
constexpr uint32_t LOG_APOGEE_INTERVAL       = HZ_TO_INTERVAL_MS(20);  // 20 Hz
constexpr uint32_t LOG_LANDED_INTERVAL      = HZ_TO_INTERVAL_MS(10);  // 10 Hz

// #define BUZZER_ON_INTERVAL        = 50ul;                    // 50 ms
// #define BUZZER_IDLE_INTERVAL      = HZ_TO_INTERVAL_MS(1);    // 1 Hz
// #define BUZZER_ARMED_INTERVAL     = HZ_TO_INTERVAL_MS(2);    // 2 Hz
// #define BUZZER_PAD_PREOP_INTERVAL = HZ_TO_INTERVAL_MS(10);   // 10 Hz
// #define BUZZER_ASCEND_INTERVAL    = HZ_TO_INTERVAL_MS(0.2);  // 0.2 Hz
// #define BUZZER_DESCEND_INTERVAL   = HZ_TO_INTERVAL_MS(1);    // 1 Hz


#define BUZZER_OFF_INTERVAL [](const uint32_t BUZZER_TOTAL_INTERVAL) -> uint32_t { return BUZZER_TOTAL_INTERVAL - BUZZER_ON_INTERVAL; };

// details::assertions 
// #define static_assert(TIME_TO_APOGEE_MAX >= TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
// #define static_assert(TIME_TO_BURNOUT_MAX >= TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");


// enum RadioMode {
//   RADIO_MODE_IDLE,
//   RADIO_MODE_TX,
//   RADIO_MODE_RX
// };
// volatile RadioMode currentMode = RADIO_MODE_IDLE;

#endif