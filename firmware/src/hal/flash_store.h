/**
 * @file    flash_store.h
 * @brief   One reserved flash sector for calibration data.
 *
 * @layer   hal (NEW module, not in Architecture v0.4: splits the flash
 *          access out of svc/calib so calib stays host-testable)
 * @core    0 (CLI command only)
 * @depends pico-sdk hardware_flash, pico_multicore (implementation only)
 * @timing  Erase 4 KB sector + program 256 B: typ. tens of ms, worst case
 *          hundreds of ms (check the clone's flash datasheet). XIP stalls on
 *          BOTH cores for the duration.
 * @errors  flash_err_t.
 * @isr     Not ISR-safe.
 * @traces  SR-F-09
 *
 * Write procedure (Architecture §7, flash writes):
 *   1. multicore_lockout_start_blocking()  (core 1 parked in RAM)
 *   2. watchdog extended for the erase
 *   3. interrupts off, erase + program, interrupts on
 *   4. watchdog restored, multicore_lockout_end_blocking()
 * The input gap during save is accepted and documented.
 */
#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    FLASH_OK = 0,
    FLASH_ARG,      /**< len > FLASH_STORE_MAX_LEN or NULL buffer.     */
    FLASH_VERIFY    /**< Read-back after program did not match.        */
} flash_err_t;

#define FLASH_STORE_MAX_LEN (256U)  /* one flash page */

/** @brief Copy len bytes from the reserved sector (XIP read, fast). */
flash_err_t flash_store_read(void *buf, size_t len);

/** @brief Erase the sector and program len bytes; verify by read-back.
 *  @pre   Called from core 0 with core 1 running (lockout is internal). */
flash_err_t flash_store_write(const void *buf, size_t len);

#endif /* FLASH_STORE_H */
