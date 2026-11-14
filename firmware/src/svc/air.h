/**
 * @file    air.h
 * @brief   Air beam decision: on-off differential, per-beam threshold with
 *          hysteresis, saturation detection.
 *
 * @layer   svc (pure logic; host unit-tested)
 * @core    1 writes (air_update from the ir_air scan callback, ISR context);
 *          core 0 reads air_read()
 * @depends none (samples passed in by value)
 * @timing  air_update() for one beam: < 2 us.
 * @errors  None.
 * @isr     air_update() is ISR-safe (no locks, no allocation).
 *          air_read()/air_saturated() are single byte reads.
 * @traces  SR-F-05, SR-F-11, SR-F-12, SR-P-04
 *
 * Decision per beam (Appendix A, steps 5-6):
 *   signal = on - off (clamped at 0)
 *   off >= sat_level              -> saturated, beam NOT blocked (SR-F-12)
 *   signal <  block_th            -> blocked
 *   signal >= block_th + hyst     -> released
 */
#ifndef AIR_H
#define AIR_H

#include <stdint.h>

#define AIR_N_BEAMS (6U)

typedef struct
{
    uint16_t block_th[AIR_N_BEAMS]; /**< ADC counts; from calibration.     */
    uint16_t hyst;                  /**< ADC counts added for release.     */
    uint16_t sat_level;             /**< OFF reading treated as saturated
                                         (e.g. 0.95 x 4095).               */
} air_config_t;

/** @brief Reset; all beams clear. */
void air_init(const air_config_t *cfg);

/** @brief Process one beam's OFF/ON pair. */
void air_update(uint8_t beam, uint16_t off, uint16_t on);

/** @brief 6-bit mask, bit n = beam n blocked (SR-F-05 air_read()). */
uint8_t air_read(void);

/** @brief 6-bit mask, bit n = beam n saturated by ambient light. */
uint8_t air_saturated(void);

/** @brief Last differential signal for one beam, for CLI/calibration. */
uint16_t air_signal(uint8_t beam);

/** @brief Replace thresholds at run time (from CLI or calib). */
void air_set_config(const air_config_t *cfg);

#endif /* AIR_H */
