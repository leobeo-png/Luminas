/**
 * @file    gpio_dbg.h
 * @brief   Debug marker pins DBG0..DBG2 for logic-analyser timing.
 *
 * @layer   hal
 * @core    any (SIO set/clr registers are atomic per core)
 * @depends pico-sdk hardware_gpio (implementation only)
 * @timing  set/clr: single SIO register write (< 10 ns at 125 MHz).
 * @isr     ISR-safe.
 * @traces  SR-I-08, SR-P-01, SR-P-04 (latency measurement)
 *
 * Compiled to no-ops when LUMINAS_DBG_MARKERS == 0, so markers can stay in
 * the code.
 */
#ifndef GPIO_DBG_H
#define GPIO_DBG_H

#include <stdint.h>

#ifndef LUMINAS_DBG_MARKERS
#define LUMINAS_DBG_MARKERS (1)
#endif

typedef enum
{
    DBG_0 = 0,  /**< Touch: I2C read with a changed mask.      */
    DBG_1,      /**< USB: HID report queued.                   */
    DBG_2       /**< Air: beam state changed.                  */
} dbg_marker_t;

/** @brief Configure DBG0..DBG2 as outputs, low. */
void gpio_dbg_init(void);

#if LUMINAS_DBG_MARKERS
void gpio_dbg_set(dbg_marker_t m);
void gpio_dbg_clr(dbg_marker_t m);
void gpio_dbg_toggle(dbg_marker_t m);
#else
#define gpio_dbg_set(m)     ((void)(m))
#define gpio_dbg_clr(m)     ((void)(m))
#define gpio_dbg_toggle(m)  ((void)(m))
#endif

#endif /* GPIO_DBG_H */
