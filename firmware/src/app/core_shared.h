/**
 * @file    core_shared.h
 * @brief   Data exchanged between core 1 (sensing) and core 0 (USB, LEDs,
 *          watchdog).
 *
 * @layer   app
 * @core    both
 * @traces  SR-P-01, SR-R-04, SR-F-08
 *
 * Rules:
 *   - Single-writer per field. Core 1 writes sensing fields, core 0 reads.
 *   - 32-bit aligned scalars only for lock-free fields (atomic on M0+).
 *   - Multi-word diagnostics go through the seqlock (seq odd = write in
 *     progress; reader retries).
 *
 * Watchdog (SR-R-04): core 0 feeds only if core1_heartbeat changed since
 * the last feed. Timeout 100 ms.
 */
#ifndef CORE_SHARED_H
#define CORE_SHARED_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t slider_mask;      /**< core 1 -> core 0              */
    volatile uint32_t slider_valid;     /**< zones served by working chips */
    volatile uint32_t air_mask;         /**< bits 0..5 (uint32 for atomics)*/
    volatile uint32_t air_saturated;    /**< bits 0..5                     */
    volatile uint32_t core1_heartbeat;  /**< ++ every core 1 loop          */
} core_live_t;

typedef struct
{
    volatile uint32_t seq;              /**< seqlock counter               */
    uint16_t filtered[3][12];
    uint16_t baseline[3][12];
    uint16_t air_off[6];
    uint16_t air_on[6];
    uint32_t i2c_timeouts;
    uint32_t i2c_nacks;
    uint32_t i2c_recoveries;
} core_diag_t;

extern core_live_t g_live;
extern core_diag_t g_diag;

#endif /* CORE_SHARED_H */
