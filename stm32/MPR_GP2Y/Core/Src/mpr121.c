/*
 * mpr121.c
 *
 *  Created on: Sep 20, 2025
 *      Author: leoar
 */

#include "mpr121.h"
#include "stm32f4xx_hal.h"

bool MPR121_begin(MPR121_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr,
uint8_t touchThresh, uint8_t releaseThresh, bool autoconfig) {

    dev->hi2c = hi2c;
    dev->i2c_addr = addr;
    dev->autoconfig = autoconfig;

    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->i2c_addr, 3, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    // Soft reset MPR121
    MPR121_writeRegister(dev, 0x80, 0x63);

    // Stop mode
    MPR121_writeRegister(dev, 0x5E, 0x00);

    // Touch pad baseline filter
    // Rising: baseline quick rising
    MPR121_writeRegister(dev, 0x2B, 1); // Max half delta Rising
    MPR121_writeRegister(dev, 0x2C, 1); // Noise half delta Rising
    MPR121_writeRegister(dev, 0x2D, 1); // Noise count limit Rising
    MPR121_writeRegister(dev, 0x2E, 1); // Delay limit Rising

    // Falling: baseline slow falling
    MPR121_writeRegister(dev, 0x2F, 1); // Max half delta Falling
    MPR121_writeRegister(dev, 0x30, 1); // Noise half delta Falling
    MPR121_writeRegister(dev, 0x31, 6); // Noise count limit Falling
    MPR121_writeRegister(dev, 0x32, 12); // Delay limit Falling

    // Touched: baseline very slow falling
    MPR121_writeRegister(dev, 0x33, 1); // Noise half delta Touched
    MPR121_writeRegister(dev, 0x34, 8); // Noise count Touched
    MPR121_writeRegister(dev, 0x35, 30); // Delay limit Touched

    // Touch pad threshold
    for (int i = 0; i < 12; i++) {
        MPR121_writeRegister(dev, 0x41 + i * 2, touchThresh);
        MPR121_writeRegister(dev, 0x42 + i * 2, releaseThresh);
    }

    // Touch and release debounce
    MPR121_writeRegister(dev, 0x5B, 0x00);

    // AFE and filter configuration
    MPR121_writeRegister(dev, 0x5C, 0b00010000); // AFES=6 samples, Global CDC=16uA
    MPR121_writeRegister(dev, 0x5D, 0b00101000); // CT=0.5us, TDS=4samples, TDI=16ms
    MPR121_writeRegister(dev, 0x5E, 0x80); // Baseline calibration enabled, load 5MSB

    // Auto Configuration
    MPR121_writeRegister(dev, 0x7B, 0b00001011); // AFES=6 samples

    // Max out sensitivity
    const uint8_t usl = (3.3 - 0.1) / 3.3 * 256;
    MPR121_writeRegister(dev, 0x7D, usl);
    MPR121_writeRegister(dev, 0x7E, usl * 0.65);
    MPR121_writeRegister(dev, 0x7F, usl * 0.9);

    // Run 12 touch, load 5MSB to baseline
    MPR121_writeRegister(dev, 0x5E, 0x8C);

    uint8_t check = MPR121_readRegister8(dev, 0x5E);
    return (check == 0x8C);
}

void MPR121_writeRegister(MPR121_t *dev, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

// Read a single 8-bit register
uint8_t MPR121_readRegister8(MPR121_t *dev, uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
    return val;
}

// Read a 16-bit register (little endian)
uint16_t MPR121_readRegister16(MPR121_t *dev, uint8_t reg) {
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    return ((uint16_t)buf[1] << 8) | buf[0];
}

// Set touch and release thresholds for all electrodes
void MPR121_setThresholds(MPR121_t *dev, uint8_t touch, uint8_t release) {
    for (uint8_t i = 0; i < 12; i++) {
        MPR121_writeRegister(dev, 0x41 + i*2, touch);
        MPR121_writeRegister(dev, 0x42 + i*2, release);
    }
}

// Return 12-bit touched status
uint16_t MPR121_touched(MPR121_t *dev) {
    return MPR121_readRegister16(dev, 0x00) & 0x0FFF;  // Only electrodes 0–11
}
