/*
 * hid_nkro.c
 *
 *  Created on: Apr 19, 2026
 *      Author: leoar
 */


#include "hid_nkro.h"
#include "usb_device.h"
#include "usbd_custom_hid_if.h"
#include <string.h>

static NKROReport nkro_report = {0};

void NKRO_PressKey(uint8_t keycode) {
	if (keycode >= 0xE0 && keycode <= 0xE7) {
		// modifier key
		nkro_report.modifiers |= (1 << (keycode & 0xE0));
	} else if (keycode <= 0x6F) {
		// regular keys
		nkro_report.keys[keycode >> 3] |= (1 << (keycode & 0x07));
	}
}

void NKRO_ReleaseKey(uint8_t keycode) {
	if (keycode >= 0xE0 && keycode <= 0xE7) {
		nkro_report.modifiers &= ~(1 << (keycode - 0xE0));
	} else if (keycode <= 0x6F) {
		nkro_report.keys[keycode >> 3] &= ~(1 << (keycode & 0x07));
	}
}

void NKRO_ReleaseAll(void) {
	memset(&nkro_report, 0, sizeof(NKROReport));
}

void NKRO_SendReport(void) {
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&nkro_report, sizeof(NKROReport));
}
