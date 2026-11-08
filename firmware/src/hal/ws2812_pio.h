/**
 * @file    ws2812_pio.h
 * @brief   WS2812B output via PIO0 SM0 + DMA, no CPU time during transfer.
 *
 * @layer   hal
 * @core    0
 * @depends pico-sdk hardware_pio, hardware_dma (implementation only)
 * @timing  31 LEDs x 24 bit x 1.25 us = 0.93 ms + >= 280 us reset latch
 *          (WS2812B-4020-V6). ws2812_pio_show() returns in < 10 us.
 * @errors  Returns false if a frame is still in flight.
 * @isr     ws2812_pio_busy() is ISR-safe; show() is not.
 * @traces  SR-F-06, SR-P-07, SR-E-04
 */
#ifndef WS2812_PIO_H
#define WS2812_PIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief  Load the PIO program, claim SM and a DMA channel.
 * @param  pin     data pin (PIN_LED_DATA).
 * @param  n_leds  number of LEDs in the chain (<= 64).
 * @return true on success; false if no SM/DMA channel is free.
 */
bool ws2812_pio_init(uint8_t pin, size_t n_leds);

/**
 * @brief  Start a DMA transfer of one frame.
 * @param  grb  n_leds words, each 0x00GGRRBB << 8 (MSB first on the wire).
 *              Buffer must stay valid until ws2812_pio_busy() is false.
 * @return true if started; false if the previous frame (incl. reset latch)
 *         has not finished.
 * @note   Brightness and current limiting are applied before this call
 *         (svc/leds); this layer sends what it is given.
 */
bool ws2812_pio_show(const uint32_t *grb, size_t n_leds);

/** @brief true while DMA runs or the reset latch time has not elapsed. */
bool ws2812_pio_busy(void);

#endif /* WS2812_PIO_H */
