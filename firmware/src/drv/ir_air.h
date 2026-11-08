/**
 * @file    ir_air.h
 * @brief   IR beam scanner: hardware-timer state machine producing raw
 *          OFF/ON ADC samples per beam (Requirements Appendix A).
 *
 * @layer   drv
 * @core    1 (timer alarm IRQ on core 1)
 * @depends hal/adc_air.h, GPIO for SEL0..2 and IR_EN
 * @timing  Per beam: mux settle (~5 us) + OFF read (2 us) + IR_EN on +
 *          settle_us (20..50 us, verify) + ON read (2 us) + IR_EN off.
 *          6 beams ~ 0.2..0.4 ms per scan; scan period default 1000 us.
 *          Each ISR step < 5 us; no busy-waiting longer than one ADC read.
 * @errors  None at run time; init returns false on bad config.
 * @isr     ir_air_get_sample() is safe from core 0 and core 1 (seqlock).
 * @traces  SR-F-05, SR-F-11, SR-F-12, SR-P-04, SR-R-07
 *
 * CHANGE vs Architecture v0.4: thresholding (blocked/not blocked) moves to
 * svc/air so it is host-testable. This driver only delivers raw samples.
 * ir_air_mask()/ir_air_saturated() from the v0.4 contract are now
 * air_read()/air_saturated() in svc/air.h.
 *
 * Safety: IR_EN is driven low on every state exit, on ir_air_stop() and
 * before init returns. Hardware pull-down covers reset/boot (SR-R-07).
 */
#ifndef IR_AIR_H
#define IR_AIR_H

#include <stdbool.h>
#include <stdint.h>

#define IR_AIR_N_BEAMS  (6U)

typedef struct
{
    uint32_t scan_period_us;   /**< Start-to-start of full scans (1000).   */
    uint16_t mux_settle_us;    /**< After SEL change, before OFF read (5). */
    uint16_t led_settle_us;    /**< After IR_EN high, before ON read.      */
} ir_air_config_t;

typedef struct
{
    uint16_t off;       /**< Ambient only (LED off).                        */
    uint16_t on;        /**< Ambient + beam (LED on).                       */
    uint32_t seq;       /**< Scan counter; increments every full scan.      */
} ir_air_sample_t;

/** @brief Configure pins (IR_EN low first), ADC, timer. Does not start. */
bool ir_air_init(const ir_air_config_t *cfg);

/** @brief Start periodic scanning (alarm on the calling core: core 1). */
void ir_air_start(void);

/** @brief Stop scanning; IR_EN low on return. */
void ir_air_stop(void);

/** @brief Latest consistent OFF/ON pair for one beam (0..5). */
void ir_air_get_sample(uint8_t beam, ir_air_sample_t *out);

/** @brief Callback after each full scan, called in ISR context on core 1.
 *         Used by main_core1 to run svc/air. NULL to disable. */
void ir_air_set_scan_callback(void (*cb)(void));

#endif /* IR_AIR_H */
