#include <Wire.h>
#include "mpr121.h"
#include "air.h"
#include "TCA9548.h"
#include "Adafruit_TinyUSB.h"
#include <VL53L0X.h>
#include <array>

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x)  //Serial.println(x)
// Remove the F() macro from printf to avoid type conversion issues
#define DEBUG_PRINTF(format, ...)  //Serial.printf(format, __VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(format, ...)
#endif

const uint16_t TOF_MIN_DISTANCE = 100;
const uint16_t TOF_MAX_DISTANCE = 340;
const uint32_t DEBOUNCE_TIME_MS = 5;
const uint32_t TOF_SAMPLE_RATE_MS = 5;
const uint16_t TOF_DIFFERENCE = 40;
const uint8_t NKRO_REPORT_SIZE = 10;  // 80 bits
const uint8_t REPORT_ID = 1;
const uint8_t TOUCH_CHANNELS = 24;       // Total touch channels (12 per sensor)
const uint8_t I2C_SWITCH_DELAY_US = 10;  // Microseconds to wait after I2C channel switch

const uint8_t LED_1 = 6;
const uint8_t LED_2 = 7;
const uint8_t LED_3 = 8;

struct KeyMapping {
  uint16_t distance_min;
  uint16_t distance_max;
  uint8_t keycode;
};

struct __attribute__((packed)) NKROReport {  // reduced bits by packing it
  uint8_t modifiers;                         // Modifier keys
  uint8_t reserved;                          // Reserved byte
  uint8_t keys[NKRO_REPORT_SIZE];            // Bitmap of keys
};

// Key state tracking
struct KeyState {
  bool is_pressed;
  uint32_t last_change;
  uint8_t source;  // 0 = cap sensor, 1 = ToF
};

const KeyMapping TOF_MAPPINGS[] = {
  { 100, 140, 0x39 },  // IR1
  { 140, 180, 0x38 },  // IR2
  { 180, 220, 0x35 },  // IR3
  { 220, 260, 0x34 },  // IR4
  { 260, 300, 0x31 },  // IR5
  { 300, 340, 0x30 }   // IR6
};

const uint8_t TOF_MAPPING_COUNT = 6;

// System status tracking
struct SystemStatus {
  bool usb_ready;
  bool cap1_ready;
  bool cap2_ready;
  bool tof1_ready;
  bool tof2_ready;
  uint32_t last_error;
};

uint8_t const desc_hid_report[] PROGMEM = {
  0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,       // USAGE (Keyboard)
  0xA1, 0x01,       // COLLECTION (Application)
  0x85, REPORT_ID,  //   Report ID (1)
  // Modifiers
  0x05, 0x07,  //   USAGE_PAGE (Keyboard)
  0x19, 0xE0,  //   USAGE_MINIMUM (Keyboard LeftControl)
  0x29, 0xE7,  //   USAGE_MAXIMUM (Keyboard Right GUI)
  0x15, 0x00,  //   LOGICAL_MINIMUM (0)
  0x25, 0x01,  //   LOGICAL_MAXIMUM (1)
  0x75, 0x01,  //   REPORT_SIZE (1)
  0x95, 0x08,  //   REPORT_COUNT (8)
  0x81, 0x02,  //   INPUT (Data,Var,Abs)
  // Reserved byte
  0x95, 0x01,  //   REPORT_COUNT (1)
  0x75, 0x08,  //   REPORT_SIZE (8)
  0x81, 0x03,  //   INPUT (Cnst,Var,Abs)
  // LED report
  0x95, 0x05,  //   REPORT_COUNT (5)
  0x75, 0x01,  //   REPORT_SIZE (1)
  0x05, 0x08,  //   USAGE_PAGE (LEDs)
  0x19, 0x01,  //   USAGE_MINIMUM (Num Lock)
  0x29, 0x05,  //   USAGE_MAXIMUM (Kana)
  0x91, 0x02,  //   OUTPUT (Data,Var,Abs)
  0x95, 0x01,  //   REPORT_COUNT (1)
  0x75, 0x03,  //   REPORT_SIZE (3)
  0x91, 0x03,  //   OUTPUT (Cnst,Var,Abs)
  // Bitmap of keys
  0x95, 0x28,  // REPORT_COUNT (40)
  0x75, 0x01,  // REPORT_SIZE (1)
  0x15, 0x00,  // LOGICAL_MINIMUM (0)
  0x25, 0x01,  // LOGICAL_MAXIMUM (1)
  0x05, 0x07,  // USAGE_PAGE (Keyboard)
  0x19, 0x00,  // USAGE_MINIMUM (0)
  0x29, 0x27,  // USAGE_MAXIMUM (39)
  0x81, 0x02,  // INPUT (Data,Var,Abs)

  0x95, 0x28,  // REPORT_COUNT (40)
  0x75, 0x01,  // REPORT_SIZE (1)
  0x15, 0x00,  // LOGICAL_MINIMUM (0)
  0x25, 0x01,  // LOGICAL_MAXIMUM (1)
  0x05, 0x07,  // USAGE_PAGE (Keyboard)
  0x19, 0x27,  // USAGE_MINIMUM (39)
  0x29, 0x4F,  // USAGE_MAXIMUM (79)
  0x81, 0x02,  // INPUT (Data,Var,Abs)
  0xc0         // END
};

// Global state variables
uint32_t lasttouched = 0;
uint32_t currtouched = 0;

Adafruit_USBD_HID usb_hid;
MPR121_LowLatency cap1 = MPR121_LowLatency();
MPR121_LowLatency cap2 = MPR121_LowLatency();
VL53L0X tof1;
VL53L0X tof2;
PCA9546 MP(0x70);  // I2C Multiplexer address
SystemStatus system_status = { false, false, false, false, 0, 0 };
NKROReport nkro_report = { 0 };
KeyState key_states[256] = { 0 };


void setup() {
  //Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  watchdog_enable(100, 1);

  if (!initializeHardware()) {
    DEBUG_PRINT(F("Hardware initialization failed. System entering safe mode."));
    while (1) {
      // Blink LED to indicate error
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }
}

bool initializeHardware() {
  // Initialize LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(1, INPUT);

  // Initialize I2C multiplexer
  if (!MP.begin()) {
    handleErrors("Multiplexer initialization failed");
    return false;
  }

  // Initialize first capacitive sensor
  if (!switchI2CChannel(0) || !cap1.begin()) {
    handleErrors("Cap1 initialization failed");
    return false;
  }
  system_status.cap1_ready = true;

  // Initialize second capacitive sensor
  if (!switchI2CChannel(1) || !cap2.begin()) {
    handleErrors("Cap2 initialization failed");
    return false;
  }
  system_status.cap2_ready = true;

  tof1.setTimeout(500);
  if (!switchI2CChannel(2) || !tof1.init()) {
    handleErrors("ToF1 initialization failed");
    return false;
  }
  tof1.startContinuous();
  system_status.tof1_ready = true;

  tof2.setTimeout(500);
  if (!switchI2CChannel(3) || !tof2.init()) {
    handleErrors("ToF2 initialization failed");
    return false;
  }
  tof2.startContinuous();
  system_status.tof2_ready = true;

  // Initialize USB HID
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  usb_hid.setBootProtocol(1);
  usb_hid.setPollInterval(5); // 5ms polling rate
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();
  system_status.usb_ready = true;

  return true;
}

bool switchI2CChannel(uint8_t channel) {
  const uint8_t MAX_RETRIES = 3;
  for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
    if (MP.selectChannel(channel)) {
      delayMicroseconds(I2C_SWITCH_DELAY_US);
      return true;
    }
    delayMicroseconds(100); // Wait before retry
  }
  return false;
}

void handleErrors(const char* error_message) {
  DEBUG_PRINT(error_message);
  system_status.last_error = millis();
}

uint8_t calculateHIDKeyCode(uint8_t i) {
  if (i < 26) {
    return 0x04 + i;  // 'a' (0x04) to 'z' (0x1D) in HID
  } else {
    return 0x1E + (i - 26);  // '1' (0x1E) to '0' (0x27) in HID
  }
}

void setKey(uint8_t key, uint8_t source) {
  uint32_t current_time = millis();
  KeyState* state = &key_states[key];

  // Basic debouncing
  if (current_time - state->last_change < DEBOUNCE_TIME_MS) {
    return;
  }

  // Only set key if it's not already pressed or comes from same source
  if (!state->is_pressed || state->source == source) {
    if (key >= 0xE0 && key <= 0xE7) {
      nkro_report.modifiers |= (1 << (key - 0xE0));
    } else {
      nkro_report.keys[key / 8] |= (1 << (key % 8));
    }

    state->is_pressed = true;
    state->last_change = current_time;
    state->source = source;

#ifdef DEBUG
    const char* source_str = (source == 0) ? "Cap" : "ToF";
    DEBUG_PRINTF("Key %s pressed: 0x%02X\n", source_str, key);
    DEBUG_PRINTF("Modifier state: 0x%02X\n", nkro_report.modifiers);
#endif
  }
}

// Function to clear a key in the NKRO report
void clearKey(uint8_t key, uint8_t source) {
  KeyState* state = &key_states[key];

  // Only clear if this source owns the key
  if (state->is_pressed && state->source == source) {
    if (key >= 0xE0 && key <= 0xE7) {
      nkro_report.modifiers &= ~(1 << (key - 0xE0));
    } else {
      nkro_report.keys[key / 8] &= ~(1 << (key % 8));
    }

    state->is_pressed = false;
    state->last_change = millis();
  }
}

// Function to clear all keys
void clearAllKeys() {
  nkro_report.modifiers = 0;
  memset(nkro_report.keys, 0, NKRO_REPORT_SIZE);
}

void process_hid() {
  static uint32_t last_report_time = 0;
  const uint32_t REPORT_INTERVAL_MS = 1;

  // Safely read touch sensors
  uint32_t touched1 = 0, touched2 = 0;

  if (system_status.cap1_ready && switchI2CChannel(0)) {
    touched1 = cap1.touched();
  }

  if (system_status.cap2_ready && switchI2CChannel(1)) {
    touched2 = cap2.touched();
  }

  currtouched = touched1 | (touched2 << 12);

  // Process touch inputs - don't clear previous states
  bool anyKeyPressed = false;

  for (uint8_t i = 0; i < TOUCH_CHANNELS; i++) {
    uint8_t keycode = calculateHIDKeyCode(i);
    if (currtouched & (1 << i)) {
      setKey(keycode, 0);  // 0 = cap sensor source
      anyKeyPressed = true;
    } else if (lasttouched & (1 << i)) {
      clearKey(keycode, 0);
    }
  }

  // Update LEDs based on touch state
  updateLEDs(currtouched);

  lasttouched = currtouched;

  // Handle USB wake-up
  if (TinyUSBDevice.suspended() && anyKeyPressed) {
    TinyUSBDevice.remoteWakeup();
  }
}


void process_tof_hid() {
  static uint32_t last_time = 0;
  static uint8_t last_codes[2] = {0, 0};
  if (millis() - last_time < TOF_SAMPLE_RATE_MS) return;
  last_time = millis();

  // Process both ToF sensors
  for (uint8_t tof_idx = 0; tof_idx < 2; tof_idx++) {
    if ((!tof_idx && system_status.tof1_ready && switchI2CChannel(2)) || 
        (tof_idx && system_status.tof2_ready && switchI2CChannel(3))) {
      uint16_t distance = (tof_idx ? tof2 : tof1).readRangeContinuousMillimeters();
      if (!(tof_idx ? tof2 : tof1).timeoutOccurred()) {
        digitalWrite(tof_idx ? LED_2 : LED_3, LOW);
        uint8_t detected_code = 0;
        
        // Find matching distance range
        for (uint8_t i = 0; i < TOF_MAPPING_COUNT && !detected_code; i++) {
          if (distance >= TOF_MAPPINGS[i].distance_min && distance < TOF_MAPPINGS[i].distance_max) {
            detected_code = TOF_MAPPINGS[i].keycode;
          }
        }

        // Update key state if changed
        if (detected_code != last_codes[tof_idx]) {
          if (last_codes[tof_idx]) clearKey(last_codes[tof_idx], tof_idx + 1);
          if (detected_code) {
            setKey(detected_code, tof_idx + 1);
            digitalWrite(tof_idx ? LED_2 : LED_3, HIGH);
          }
          last_codes[tof_idx] = detected_code;
        }
      } else {
        DEBUG_PRINT(tof_idx ? "ToF2 timeout" : "ToF1 timeout");
      }
    }
  }
}


void updateLEDs(uint32_t touch_state) {
  digitalWrite(LED_1, (touch_state & 0x3FF) ? HIGH : LOW);       // First 10 touchpoints
  digitalWrite(LED_2, (touch_state & 0xFFFFF400) ? HIGH : LOW);  // Remaining touchpoints
}

void clearLEDs() {
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
}

void loop() {
  static uint32_t last_report_time = 0;
  const uint32_t REPORT_INTERVAL_MS = 1;
  watchdog_update();
  if (!TinyUSBDevice.mounted()) {
    return;
  }

  // Process inputs
  process_hid();
  process_tof_hid();

  

  // Send HID report at regular intervals - only one place
  uint32_t current_time = millis();
  if (usb_hid.ready() && system_status.usb_ready && current_time - last_report_time >= REPORT_INTERVAL_MS) {
    bool success = usb_hid.sendReport(REPORT_ID, &nkro_report, sizeof(NKROReport));
#ifdef DEBUG
    if (!success) {
      DEBUG_PRINT("HID Report failed to send\n");
    }
#endif
    last_report_time = current_time;
  }

#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
}
