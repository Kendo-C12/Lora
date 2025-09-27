#include "LoopTime.h"

// -------- arrays methods --------
void LoopTime::arrays::push(uint32_t in) {
    data[index] = in;
    index++;
    if(index > maxLen - 1) index = 0;
    if(len < maxLen) len++;
}

uint32_t LoopTime::arrays::sum() {
    sumNum = 0;
    for(uint8_t i = 0; i < len; i++) {
        sumNum += data[i] / len;
    }
    return sumNum;
}

// -------- LoopTime methods --------
void LoopTime::print() {
    if(millis() > printT) {
        Serial.print("TIME: ");
        Serial.println(millis() - start);
        Serial.print("Duration: ");
        Serial.println(duration.sum());
        Serial.print("LoRa Time On Air: ");
        Serial.println(loraTxTimeOnAir.sum());
        Serial.println();
        printT = millis() + 100;
    }
}

void LoopTime::begin() {
    start = millis();
    tx_flag = false;
}

bool LoopTime::tx_flag_get() {
    return tx_flag;
}

bool LoopTime::transmitOrNot() {
    if(tx_flag) {
        print();
        if(millis() - start >= duration.sum() - 10) {
            start = millis();
        }

        if( ((millis() - start > loraTxTimeOnAir.sum() + 100) &&
             (duration.sum() - (millis() - start) > 100)) 
             || duration.len == 0 
             || millis() - startReal > duration.sum() * 2) 
        {
            Serial.print("Wait Time: ");
            Serial.println(millis() - startTx);
            tx_flag = false;
            return true;
        }
    }
    return false;
}

void LoopTime::transmit() {
    startTx = millis();
    tx_flag = true;
}

void LoopTime::receive(uint32_t timeOnAir) {
    if(tx_flag) return;
    if(!startOrNot) {
        startOrNot = true;
        start = millis();
        startReal = millis();
        return;
    }
    duration.push(millis() - start);
    loraTxTimeOnAir.push(10 + timeOnAir / 1000);
    start = millis();
    startReal = millis();
}
