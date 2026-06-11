/*
 * mpr121_handler.h
 *
 *  Created on: Apr 19, 2026
 *      Author: leoar
 */

#ifndef INC_MPR121_HANDLER_H_
#define INC_MPR121_HANDLER_H_

#include "mpr121.h"
#include "hid_nkro.h"

#define TOUCH_CHANNELS 36

void MPR121_Handler_Init(void);
void MPR121_Handler_Process(void);

#endif /* INC_MPR121_HANDLER_H_ */
