/**
 * @file    adc_air.h
 * @brief   Single-shot ADC reads for the IR receiver mux (ADC0) and the
 *          optional 5V_SW sense divider (ADC1).
 *
 * @layer   hal
 * @core    1 (called from the ir_air timer ISR)
 * @depends pico-sdk hardware_adc (implementation only)
 * @timing  One conversion: 96 ADC clocks at 48 MHz = 2 us.
 * @errors  None; returns raw 12-bit value.
 * @isr     ISR-safe. Core 1 owns the ADC; core 0 must not call it.
 * @traces  SR-F-05, SR-F-12, SR-P-04
 */
#ifndef ADC_AIR_H
#define ADC_AIR_H

#include <stdint.h>

#define ADC_AIR_FULL_SCALE  (4095U)

/** @brief Init ADC, configure GP26 (and GP27) as analog inputs. */
void adc_air_init(void);

/** @brief Blocking read of AIR_OUT (ADC0), ~2 us. Range 0..4095. */
uint16_t adc_air_read(void);

/** @brief Blocking read of V5SW_SENSE (ADC1), ~2 us. Range 0..4095. */
uint16_t adc_air_read_v5sw(void);

#endif /* ADC_AIR_H */
