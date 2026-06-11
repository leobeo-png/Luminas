/*
 * mpr121.h
 *
 *  Created on: Sep 20, 2025
 *      Author: leoar
 */

#ifndef INC_MPR121_H_
#define INC_MPR121_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define MPR121_ADDR_DEFAULT (0x5A << 1)
#define MPR121_TOUCH_THRESHOLD 12
#define MPR121_RELEASE_THRESHOLD 6

enum {
	  MPR121_TOUCHSTATUS_L = 0x00,
	  MPR121_TOUCHSTATUS_H = 0x01,
	  MPR121_FILTDATA_0L = 0x04,
	  MPR121_FILTDATA_0H = 0x05,
	  MPR121_BASELINE_0 = 0x1E,
	  MPR121_MHDR = 0x2B,
	  MPR121_NHDR = 0x2C,
	  MPR121_NCLR = 0x2D,
	  MPR121_FDLR = 0x2E,
	  MPR121_MHDF = 0x2F,
	  MPR121_NHDF = 0x30,
	  MPR121_NCLF = 0x31,
	  MPR121_FDLF = 0x32,
	  MPR121_NHDT = 0x33,
	  MPR121_NCLT = 0x34,
	  MPR121_FDLT = 0x35,

	  MPR121_TOUCHTH_0 = 0x41,
	  MPR121_RELEASETH_0 = 0x42,
	  MPR121_DEBOUNCE = 0x5B,
	  MPR121_CONFIG1 = 0x5C,
	  MPR121_CONFIG2 = 0x5D,
	  MPR121_CHARGECURR_0 = 0x5F,
	  MPR121_CHARGETIME_1 = 0x6C,
	  MPR121_ECR = 0x5E,
	  MPR121_AUTOCONFIG0 = 0x7B,
	  MPR121_AUTOCONFIG1 = 0x7C,
	  MPR121_UPLIMIT = 0x7D,
	  MPR121_LOWLIMIT = 0x7E,
	  MPR121_TARGETLIMIT = 0x7F,

	  MPR121_GPIODIR = 0x76,
	  MPR121_GPIOEN = 0x77,
	  MPR121_GPIOSET = 0x78,
	  MPR121_GPIOCLR = 0x79,
	  MPR121_GPIOTOGGLE = 0x7A,

	  MPR121_SOFTRESET = 0x80,
};

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_addr;
	bool autoconfig;
} MPR121_t;

// Initialize MPR121
bool MPR121_begin(MPR121_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr,
                  uint8_t touchThresh, uint8_t releaseThresh, bool autoconfig);

// Read/write registers
uint8_t MPR121_readRegister8(MPR121_t *dev, uint8_t reg);
uint16_t MPR121_readRegister16(MPR121_t *dev, uint8_t reg);
void MPR121_writeRegister(MPR121_t *dev, uint8_t reg, uint8_t value);

// Touch functions
uint16_t MPR121_touched(MPR121_t *dev);
void MPR121_setThresholds(MPR121_t *dev, uint8_t touch, uint8_t release);


#endif /* INC_MPR121_H_ */
