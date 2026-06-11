/*
 * hid_nkro.h
 *
 *  Created on: Apr 19, 2026
 *      Author: leoar
 */

#ifndef INC_HID_NKRO_H_
#define INC_HID_NKRO_H_

#pragma once
#include "main.h"
#include "usbd_custom_hid_if.h"
#include <string.h>

typedef struct {
    uint8_t modifiers;
    uint8_t keys[14];
} NKROReport;

void NKRO_PressKey(uint8_t keycode);
void NKRO_ReleaseKey(uint8_t keycode);
void NKRO_ReleaseAll(void);
void NKRO_SendReport(void);

#endif /* INC_HID_NKRO_H_ */
