// MPR121_LowLatency.cpp
#include "mpr121.h"

MPR121_LowLatency::MPR121_LowLatency(uint8_t address) : 
    _addr(address), 
    _touchData(0), 
    _lastTouchData(0) 
{
}

bool MPR121_LowLatency::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t MPR121_LowLatency::readRegister(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.read();
}

bool MPR121_LowLatency::begin(uint8_t touchThreshold, uint8_t releaseThreshold) {
    Wire.begin();
    
    // Soft reset
    writeRegister(MPR121_ECR, 0x00);
    
    // Optimize configuration for low latency
    writeRegister(MPR121_CONFIG1, 0x10); // Reduce charge time
    writeRegister(MPR121_CONFIG2, 0x20); // Increase sampling rate
    
    // Set touch and release thresholds for all electrodes
    for (uint8_t i = 0; i < 12; i++) {
        writeRegister(MPR121_TOUCH_THRESHOLD + i * 2, touchThreshold);
        writeRegister(MPR121_RELEASE_THRESHOLD + i * 2, releaseThreshold);
    }
    
    // Enable electrodes and set baseline tracking
    writeRegister(MPR121_ECR, 0x8F); // Enable first 12 electrodes with fast baseline
    
    return true;
}

bool MPR121_LowLatency::getTouchData() {
    _lastTouchData = _touchData;
    
    Wire.beginTransmission(_addr);
    Wire.write(MPR121_TOUCHSTATUS_L);
    Wire.endTransmission(false);
    
    Wire.requestFrom(_addr, (uint8_t)2);
    _touchData = Wire.read();
    _touchData |= (Wire.read() << 8);
    
    return true;
}

bool MPR121_LowLatency::isTouched(uint8_t electrode) {
    if (electrode > 11) return false;
    return (_touchData & (1 << electrode));
}

bool MPR121_LowLatency::wasJustTouched(uint8_t electrode) {
    if (electrode > 11) return false;
    return ((_touchData & (1 << electrode)) && !(_lastTouchData & (1 << electrode)));
}

bool MPR121_LowLatency::wasJustReleased(uint8_t electrode) {
    if (electrode > 11) return false;
    return (!(_touchData & (1 << electrode)) && (_lastTouchData & (1 << electrode)));
}

uint16_t MPR121_LowLatency::touched() {
    return _touchData;
}

void MPR121_LowLatency::setThresholds(uint8_t touch, uint8_t release, uint8_t electrode) {
    if (electrode == 0xFF) {
        // Set all electrodes
        for (uint8_t i = 0; i < 12; i++) {
            writeRegister(MPR121_TOUCH_THRESHOLD + i * 2, touch);
            writeRegister(MPR121_RELEASE_THRESHOLD + i * 2, release);
        }
    } else if (electrode < 12) {
        // Set single electrode
        writeRegister(MPR121_TOUCH_THRESHOLD + electrode * 2, touch);
        writeRegister(MPR121_RELEASE_THRESHOLD + electrode * 2, release);
    }
}