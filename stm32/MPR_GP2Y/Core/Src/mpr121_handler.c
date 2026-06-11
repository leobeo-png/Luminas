/*
 * mpr121_handler.c
 *
 *  Created on: Apr 19, 2026
 *      Author: leoar
 */


#include "mpr121_handler.h"

extern I2C_HandleTypeDef hi2c1;

static MPR121_t cap1, cap2, cap3;
static uint64_t lastTouched = 0;

static const uint8_t keycode_map[TOUCH_CHANNELS] = {
	// cap1 = 0x5B (channels 0-11)
	0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
	0x14, 0x15, 0x16, 0x17, 0x18, 0x19,

	// cap2 = 0x5C (channels 12-23)
	0x26, 0x27, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,

    // cap3 = 0x5A (channels 24-35)
	0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x3A, 0x3B, 0x3C, 0x3D,

};

void MPR121_Handler_Init(void) {
    MPR121_begin(&cap1, &hi2c1, (0x5B << 1), 20, 10, true);
    MPR121_begin(&cap2, &hi2c1, (0x5C << 1), 12, 6, true);
    MPR121_begin(&cap3, &hi2c1, (0x5A << 1), 12, 6, true);
}

void MPR121_Handler_Process(void) {
	uint64_t currentTouched = (uint64_t)MPR121_touched(&cap1)
							| ((uint64_t)MPR121_touched(&cap2) << 12)
							| ((uint64_t)MPR121_touched(&cap3) << 24);

	if (currentTouched == lastTouched) return;

	uint64_t changed = currentTouched ^ lastTouched;

	for (uint8_t i = 0; i < TOUCH_CHANNELS; i++) {
		uint64_t bit = (1ULL << i);
		if (changed & bit) {
			uint8_t keycode = keycode_map[i];
			if (currentTouched & bit){
				NKRO_PressKey(keycode);
			} else {
				NKRO_ReleaseKey(keycode);
			}
		}
	}

	lastTouched = currentTouched;
}
