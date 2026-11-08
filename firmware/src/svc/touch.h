/**
 * @file    touch.h
 * @brief   32-zone slider state: chip/electrode -> zone mapping, debounce,
 *          per-chip validity.
 *
 * @layer   svc (pure logic, no pico-sdk includes; host unit-tested)
 * @core    1 writes (touch_update); core 0 reads touch_mask()
 * @depends none
 * @timing  touch_update(): < 5 us on Cortex-M0+ at 125 MHz.
 * @errors  None; invalid arguments are ignored (and counted).
 * @isr     touch_mask() is a single aligned 32-bit read: safe from core 0.
 * @traces  SR-F-01, SR-F-02, SR-P-05, SR-R-03
 *
 * Touch/release thresholds with hysteresis are done in the MPR121 itself
 * (registers, SR-F-03); this layer only debounces and maps.
 *
 * Mapping (to confirm in schematic): 3 chips x 12 electrodes = 36 inputs,
 * 32 used. Zone n = column (n / 2), row (n % 2), RedBoard bit order.
 * Table lives in touch.c as touch_map[chip][electrode] -> zone or 0xFF.
 */
#ifndef TOUCH_H
#define TOUCH_H

#include <stdbool.h>
#include <stdint.h>

#define TOUCH_N_CHIPS   (3U)
#define TOUCH_N_ZONES   (32U)
#define TOUCH_ZONE_NONE (0xFFU)

typedef struct
{
    uint32_t debounce_us;   /**< Press must be stable this long. Must be
                                 <= 1000 (SR-P-05). 0 = off.               */
} touch_config_t;

/** @brief Reset state; all zones released, all chips valid. */
void touch_init(const touch_config_t *cfg);

/**
 * @brief  Feed a fresh 12-bit mask from one chip.
 * @param  chip     0..2
 * @param  raw_mask bits 0..11 from mpr121_read_touch()
 * @param  now_us   monotonic time (time_us_32 on target)
 * @note   Release is reported immediately; press is debounced
 *         (old bug: debounce dropped presses permanently).
 */
void touch_update(uint8_t chip, uint16_t raw_mask, uint32_t now_us);

/** @brief Mark a chip failed/recovered. Failed chip's zones read as released
 *         and are excluded from touch_valid_mask() (SR-R-03). */
void touch_set_chip_valid(uint8_t chip, bool valid);

/** @brief Debounced 32-zone mask, bit n = zone n touched. */
uint32_t touch_mask(void);

/** @brief Bit n = zone n is served by a working chip. */
uint32_t touch_valid_mask(void);

/** @brief Zone for (chip, electrode), or TOUCH_ZONE_NONE. For CLI. */
uint8_t touch_zone_of(uint8_t chip, uint8_t electrode);

#endif /* TOUCH_H */
