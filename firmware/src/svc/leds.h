/**
 * @file    leds.h
 * @brief   LED frame builder: host BRG data or local fallback, brightness,
 *          and a current limiter against the USB budget.
 *
 * @layer   svc (pure logic; host unit-tested)
 * @core    0
 * @depends none (output passed to hal/ws2812_pio by main_core0)
 * @timing  leds_build_frame(): 31 LEDs, < 50 us.
 * @errors  None; out-of-range values are clamped.
 * @isr     Not called from ISRs.
 * @traces  SR-F-06, SR-F-07, SR-E-01, SR-E-02, SR-P-07
 *
 * Current model (Architecture §5): I = sum(R+G+B)/255 x ma_per_channel.
 * The frame is scaled down uniformly until I <= allowance_ma (313 mA in
 * power budget v1). Use the per-channel current measured in Phase 6.
 */
#ifndef LEDS_H
#define LEDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LEDS_N              (31U)           /* 16 keys + 15 dividers      */
#define LEDS_BRG_BYTES      (LEDS_N * 3U)   /* 93, SR-I-07                */
#define LEDS_MIN_PCT        (5U)            /* SR-E-02                    */

typedef struct
{
    uint16_t ma_per_channel_x10;  /**< Full-on current per channel, 0.1 mA
                                       units (120 = 12.0 mA, datasheet typ). */
    uint16_t allowance_ma;        /**< Budget for LED drive (313).          */
    uint8_t  max_pct;             /**< Upper brightness clamp from measured
                                       current (SR-E-02).                   */
    uint32_t host_timeout_us;     /**< No host frame this long -> local
                                       touch feedback (SR-F-07, 1 s).       */
} leds_config_t;

void leds_init(const leds_config_t *cfg);

/** @brief Store host colours (BRG order, as RedBoard sends them).
 *  @param len must be LEDS_BRG_BYTES; other lengths are ignored. */
void leds_set_brg(const uint8_t *brg, size_t len, uint32_t now_us);

/** @brief User brightness, clamped to LEDS_MIN_PCT..max_pct. */
void leds_set_brightness_pct(uint8_t pct);

/** @brief Build the output frame (GRB words for ws2812_pio_show()).
 *  @param touch_mask used for local feedback when the host is silent.
 *  @param grb_out    LEDS_N words. */
void leds_build_frame(uint32_t touch_mask, uint32_t now_us,
                      uint32_t *grb_out);

/** @brief Estimated current of the last built frame, mA. */
uint32_t leds_estimate_ma(void);

/** @brief true if host data arrived within host_timeout_us. */
bool leds_host_active(uint32_t now_us);

#endif /* LEDS_H */
