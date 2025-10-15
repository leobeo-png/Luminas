// MPR121_LowLatency.h
#ifndef MPR121_LOWLATENCY_H
#define MPR121_LOWLATENCY_H

#include <Wire.h>

class MPR121_LowLatency {
private:
    uint8_t _addr;
    uint16_t _touchData;
    uint16_t _lastTouchData;
    
    // Register addresses
    static const uint8_t MPR121_TOUCHSTATUS_L = 0x00;
    static const uint8_t MPR121_CONFIG1 = 0x5C;
    static const uint8_t MPR121_CONFIG2 = 0x5D;
    static const uint8_t MPR121_ECR = 0x5E;
    static const uint8_t MPR121_TOUCH_THRESHOLD = 0x41;
    static const uint8_t MPR121_RELEASE_THRESHOLD = 0x42;
    
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

public:
    MPR121_LowLatency(uint8_t address = 0x5A);
    bool begin(uint8_t touchThreshold = 12, uint8_t releaseThreshold = 6);
    bool getTouchData();
    bool isTouched(uint8_t electrode);
    bool wasJustTouched(uint8_t electrode);
    bool wasJustReleased(uint8_t electrode);
    uint16_t touched();
    void setThresholds(uint8_t touch, uint8_t release, uint8_t electrode = 0xFF);
};

#endif