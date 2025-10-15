#include <HID-Project.h>
#include "Adafruit_MPR121.h"
#include "TCA9548.h"
#include <VL53L0X.h>

const uint8_t KEYCODES[6] = { 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F };  // F1-F6
const uint16_t THRESHOLDS[5] = { 100, 200, 300, 400, 500 };
const uint8_t I2C_DELAY_US = 100;
const uint8_t TOF_POLL_MS = 5;

// Pro Micro I2C pins
const uint8_t SDA_PIN = 2;
const uint8_t SCL_PIN = 3;

uint8_t lastKeys[2] = { 0, 0 };
Adafruit_MPR121 cap1, cap2, cap3;
VL53L0X tof1, tof2;
PCA9546 MP(0x70);
uint64_t lastTouched = 0;

void hardRecoverI2C() {
  Wire.end();
  
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  
  // Send 10 clock pulses to free stuck slave
  for (int i = 0; i < 10; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
  }
  
  Wire.begin();
  Wire.setClock(100000);
  delay(10);
}

void assertI2CFunctions() {
  // Test if multiplexer (0x70) ACKs
  Wire.beginTransmission(0x70);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    hardRecoverI2C();
    return;
  }
  
  // Test if unused address (0x03) does NOT ACK
  Wire.beginTransmission(0x03);
  error = Wire.endTransmission();
  
  if (error == 0) {
    hardRecoverI2C();
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  
  // Initial I2C bus check
  assertI2CFunctions();
  
  cap1.begin(0x5A);
  cap2.begin(0x5C);
  cap3.begin(0x5B);
  MP.begin();
  
  if (MP.selectChannel(0)) {
    tof1.init();
    tof1.startContinuous();
  }
  if (MP.selectChannel(3)) {
    tof2.init();
    tof2.startContinuous();
  }
  
  NKROKeyboard.begin();
}

uint8_t mapKeyCode(uint8_t i) {
  if (i < 8) return 0x30 + i;              // '0'-'7'
  if (i < 12) return 0xC2 + (i - 8);       // F1-F4
  if (i < 14) return 0x38 + (i - 12);      // '8'-'9'
  return 0x41 + (i - 14);                  // 'A'-'V'
}

inline bool switchI2CChannel(uint8_t channel) {
  if (MP.selectChannel(channel)) {
    delayMicroseconds(I2C_DELAY_US);
    return true;
  }
  return false;
}

uint8_t mapDistance(uint16_t d) {
  if (d < 100 || d >= 500) return 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (d < THRESHOLDS[i]) return KEYCODES[i];
  }
  return 0;
}

void processTouchSensors() {
  uint64_t currentTouched = cap1.touched() |
                           ((uint64_t)cap2.touched() << 12) |
                           ((uint64_t)cap3.touched() << 24);
  
  uint64_t changed = currentTouched ^ lastTouched;
  
  if (changed) {
    for (uint8_t i = 0; i < 36; i++) {
      uint64_t bit = (1ULL << i);
      if (changed & bit) {
        uint8_t keycode = mapKeyCode(i);
        if (currentTouched & bit) {
          NKROKeyboard.add(keycode);
        } else {
          NKROKeyboard.remove(keycode);
        }
      }
    }
    lastTouched = currentTouched;
  }
}

void processToFSensors() {
  static uint32_t lastTofTime = 0;
  uint32_t now = millis();
  
  if (now - lastTofTime < TOF_POLL_MS) return;
  lastTofTime = now;
  
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t channel = i ? 3 : 0;
    if (!switchI2CChannel(channel)) continue;
    
    VL53L0X& sensor = i ? tof2 : tof1;
    uint16_t d = sensor.readRangeContinuousMillimeters();
    
    if (!sensor.timeoutOccurred()) {
      uint8_t key = mapDistance(d);
      if (key != lastKeys[i]) {
        if (lastKeys[i]) NKROKeyboard.remove(lastKeys[i]);
        if (key) NKROKeyboard.add(key);
        lastKeys[i] = key;
      }
    } else if (lastKeys[i]) {
      NKROKeyboard.remove(lastKeys[i]);
      lastKeys[i] = 0;
    }
  }
}

void loop() {
  static uint32_t lastI2CCheck = 0;
  
  // Check I2C health every 100ms
  if (millis() - lastI2CCheck > 100) {
    assertI2CFunctions();
    lastI2CCheck = millis();
  }
  
  processTouchSensors();
  processToFSensors();
  NKROKeyboard.send();
}