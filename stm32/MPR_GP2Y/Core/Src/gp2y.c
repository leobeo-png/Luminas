/*
 * gp2y.c
 *
 *  Created on: Apr 19, 2026
 *      Author: leoar
 */


#include "gp2y.h"

#define GP2Y_SENSOR_COUNT 2
#define GP2Y_SAMPLES 16
#define HYSTERESIS 50  // raw ADC units
extern ADC_HandleTypeDef hadc1;

volatile uint32_t adc_buf[GP2Y_SENSOR_COUNT];
static uint8_t lastKeys[GP2Y_SENSOR_COUNT] = {0, 0};
HAL_StatusTypeDef gp2y_dma_status;

static uint8_t mapDistance(uint32_t raw, uint8_t sensor_idx) {
    static uint32_t last_raw[2] = {0, 0};

    if (raw > last_raw[sensor_idx] - HYSTERESIS &&
        raw < last_raw[sensor_idx] + HYSTERESIS) {
        raw = last_raw[sensor_idx];
    }
    last_raw[sensor_idx] = raw;

    if (raw < 1000) return 0;
    if (raw > 2500) return 0;

    if (raw < 1250) return 0x36;    // ,
    if (raw < 1500) return 0x37;    // .
    if (raw < 1750) return 0x33;    // ;
    if (raw < 2000) return 0x34;    // '
    if (raw < 2250) return 0x2F;    // [
    return 0x30;                    // ]
}

void GP2Y_Init(void) {
    gp2y_dma_status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, GP2Y_SENSOR_COUNT);
}

static uint32_t smooth(uint8_t sensor_idx, uint32_t new_val) { // moving average filter
    static uint32_t buf[2][GP2Y_SAMPLES] = {0};
    static uint8_t idx[2] = {0};

    buf[sensor_idx][idx[sensor_idx]] = new_val;
    idx[sensor_idx] = (idx[sensor_idx] + 1) % GP2Y_SAMPLES;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < GP2Y_SAMPLES; i++) sum += buf[sensor_idx][i];
    return sum / GP2Y_SAMPLES;
}

void GP2Y_Process(void) {
    for (uint8_t i = 0; i < GP2Y_SENSOR_COUNT; i++) {
        uint8_t key = mapDistance(smooth(i, adc_buf[i]), i);

        if (key != lastKeys[i]) {
            if (lastKeys[i]) NKRO_ReleaseKey(lastKeys[i]);
            if (key)         NKRO_PressKey(key);
            lastKeys[i] = key;
        }
    }
}
