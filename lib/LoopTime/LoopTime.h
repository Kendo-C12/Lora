#ifndef LOOPTIME_H
#define LOOPTIME_H

#include <Arduino.h>

struct LoopTime {

    bool startOrNot = false;
    uint32_t start;
    uint32_t printT = millis();
    uint32_t startTx = millis();
    bool tx_flag;

    struct arrays {
        uint8_t maxLen = 9;
        uint32_t data[9] = {0};
        uint8_t index = 0;
        uint8_t len = 0;
        uint32_t sumNum = 0;

        void push(uint32_t in);
        uint32_t sum();
    } loraTxTimeOnAir, duration;

    void print();
    void begin();
    bool tx_flag_get();
    bool transmitOrNot();
    void transmit();
    void receive(uint32_t timeOnAir);
};

extern LoopTime rxLoopTime;

#endif // LOOPTIME_H
