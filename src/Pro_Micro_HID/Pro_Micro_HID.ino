#include <HID-Project.h>
#include "Adafruit_MPR121.h"
#include "TCA9548.h"
#include <VL53L0X.h>

#ifndef _BV // bitmasking single bit
#define _BV(bit) (1 << (bit)) 
#endif

const uint8_t KEYCODES[6] = {0x33, 0x34, 0x35, 0x36, 0x37, 0x38}; // '3' to '8'
const uint16_t THRESHOLDS[5] = {100, 200, 300, 400, 500}; // distance zones
uint8_t lastKeys[2] = {0, 0};

Adafruit_MPR121 cap1 = Adafruit_MPR121();
Adafruit_MPR121 cap2 = Adafruit_MPR121();
VL53L0X tof1;
VL53L0X tof2;
PCA9546 MP(0x70);

uint16_t lastTouched = 0;
uint16_t currentTouched = 0;

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000); // max for the MPR121
  // while (!Serial) { //boot delay
  //   delay(10);
  // }

  if(!cap1.begin(0x5A)) {
    Serial.println("MPR121 0x5A not found?");
    delay(10);
  }
  
  if(!cap2.begin(0x5B)) {
    Serial.println("MPR121 0x5B not found?");
    delay(10);
  }

  if(!MP.begin()){
    Serial.println("Multiplexer not found?");
  }

  if(!MP.selectChannel(0) || !tof1.init()) {
    Serial.println("tof1 not found?");
  }
  tof1.startContinuous();

  if(!MP.selectChannel(1) || !tof2.init()) {
    Serial.println("tof2 not found?");
  }
  tof2.startContinuous();

  NKROKeyboard.releaseAll();
  lastTouched = currentTouched;
}

uint8_t mapKeyCode(int i) {
  return (i < 26) ? (0x41 + i) : (0x34 + i - 26);
}

bool switchI2CChannel(uint8_t channel) {
  const uint8_t MAX_RETRIES = 3;
  for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
    if (MP.selectChannel(channel)) {
      delayMicroseconds(10);
      return true;
    }
    delayMicroseconds(100); // Wait before retry
  }
  return false;
}

uint8_t mapDistance(uint16_t d) {
  if (d < 100 || d >= 500) return 0; // No key outside range
  for (uint8_t i = 0; i < 5; i++) {
    if (d < THRESHOLDS[i]) return KEYCODES[i];
  }
  return 0;
}


void loop() {
  currentTouched = cap1.touched();
  currentTouched |= (uint32_t(cap2.touched() << 12));
//  currentTouched |= (uint32_t(cap3.touched() << 24));

  for(uint8_t i = 0; i < 32; i++){
    if((currentTouched & _BV(i)) && !(lastTouched & _BV(i))) {
      NKROKeyboard.add(mapKeyCode(i));
      Serial.println(mapKeyCode(i));
    }
    if(!(currentTouched & _BV(i)) && (lastTouched & _BV(i))) {
      NKROKeyboard.remove(mapKeyCode(i));
    }
  }

 for (uint8_t i = 0; i < 2; i++) {
  if ((!i && switchI2CChannel(0)) || (i && switchI2CChannel(1))) {
    uint16_t d = (i ? tof2 : tof1).readRangeContinuousMillimeters();
    if (!(i ? tof2 : tof1).timeoutOccurred()) {
      uint8_t key = mapDistance(d);
        // Only release if the key has changed
        if (key != lastKeys[i]) {
          if (lastKeys[i]) {
            NKROKeyboard.remove(lastKeys[i]);
          }
          lastKeys[i] = key;
        }

        if (key) {
          NKROKeyboard.add(key);  // <- Add every time
        }
      } else {
        if (lastKeys[i]) {
          NKROKeyboard.remove(lastKeys[i]);
          lastKeys[i] = 0;
        }
      }
    }
  }    


  NKROKeyboard.send();
  lastTouched = currentTouched;
  currentTouched = 0;
}
