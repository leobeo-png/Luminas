/**
 * @file    mpr121.h
 * @brief   MPR121 capacitive touch controller driver.
 *
 * @layer   drv
 * @core    1
 * @depends hal/i2c_bus.h; IRQ pin read via a function pointer so the driver
 *          stays host-testable.
 * @timing  mpr121_read_touch(): one 2-byte read, ~115 us at 400 kHz.
 *          mpr121_init(): soft reset + ~20 register writes, < 5 ms.
 * @errors  mpr121_err_t; the underlying bus_err_t is kept in
 *          dev->last_bus_err for diagnostics.
 * @isr     Not ISR-safe (uses i2c_bus).
 * @traces  SR-F-01, SR-F-02, SR-F-03, SR-F-04, SR-P-02, SR-P-08, SR-R-02,
 *          SR-R-03
 *
 * Fixes versus the old Arduino branch (Plan, Appendix B):
 *   - real soft reset (0x80 <- 0x63), not just ECR = 0
 *   - read errors returned, never turned into a "touched" mask
 *   - touch status masked to 0x0FFF
 *   - baseline filter registers 0x2B..0x40 configured explicitly
 *   - the driver never initialises the bus it uses
 */
#ifndef MPR121_H
#define MPR121_H

#include <stdbool.h>
#include <stdint.h>

#include "hal/i2c_bus.h"

#define MPR121_N_ELECTRODES     (12U)
#define MPR121_TOUCH_MASK       (0x0FFFU)
#define MPR121_IO_TIMEOUT_US    (1000U)     /* SR-R-01 TBD value: 1 ms */

typedef enum
{
    MPR121_OK = 0,
    MPR121_ERR_BUS,     /**< I2C error; see dev->last_bus_err.            */
    MPR121_ERR_ID,      /**< Post-reset register check failed (wrong chip,
                             bad ADDR strap).                             */
    MPR121_ERR_OVCF,    /**< Over-current flag set (REXT/electrode short);
                             chip stops scanning until re-init.           */
    MPR121_ERR_ARG
} mpr121_err_t;

/** Returns the IRQ pin level: true = low (interrupt pending). */
typedef bool (*mpr121_irq_fn_t)(uint8_t pin);

/** Tuning values; defaults from AN3891/AN3863, tuned in Phase 6. */
typedef struct
{
    uint8_t touch_th[MPR121_N_ELECTRODES];   /**< Touch threshold.         */
    uint8_t release_th[MPR121_N_ELECTRODES]; /**< Release (< touch, SR-F-03). */
    uint8_t cdc;        /**< CONFIG1: charge current (0x5C), 0 = auto.     */
    uint8_t cdt;        /**< CONFIG2: charge time, sample interval (0x5D). */
    uint8_t ffi;        /**< CONFIG1: first filter iterations.             */
    uint8_t sfi;        /**< CONFIG2: second filter iterations.            */
    uint8_t debounce;   /**< 0x5B: touch/release debounce (SR-P-08 cost).  */
    uint8_t n_enabled;  /**< Electrodes enabled in ECR (1..12).            */
} mpr121_config_t;

typedef struct
{
    i2c_bus_t      *bus;
    uint8_t         addr;
    uint8_t         irq_pin;
    mpr121_irq_fn_t irq_read;
    bool            online;     /**< false after an error until re-init.  */
    bus_err_t       last_bus_err;
} mpr121_t;

/** @brief Fill cfg with safe defaults (all electrodes, auto-config). */
void mpr121_config_default(mpr121_config_t *cfg);

/**
 * @brief  Soft reset, verify reset values, write config, enter run mode.
 * @pre    bus initialised by the caller.
 * @post   dev->online == (return == MPR121_OK).
 */
mpr121_err_t mpr121_init(mpr121_t *dev, i2c_bus_t *bus, uint8_t addr,
                      uint8_t irq_pin, mpr121_irq_fn_t irq_read,
                      const mpr121_config_t *cfg);

/**
 * @brief  Read touch status registers 0x00/0x01.
 * @param  mask  out: bit n = electrode n touched, masked to 0x0FFF.
 *               Unchanged on error.
 * @post   On error: dev->online = false; caller (main_core1) decides on
 *         i2c_bus_recover() + mpr121_init() and touch_set_chip_valid().
 *         OVCF (status bit 15) -> MPR121_ERR_OVCF.
 */
mpr121_err_t mpr121_read_touch(mpr121_t *dev, uint16_t *mask);

/** @brief true if the IRQ line is low (new data). Polling fallback: caller
 *         may ignore this and read every cycle (D5). */
bool mpr121_irq_pending(const mpr121_t *dev);

/** @brief Read filtered data (10-bit) for one electrode, for CLI (SR-F-08). */
mpr121_err_t mpr121_read_filtered(mpr121_t *dev, uint8_t ele, uint16_t *value);

/** @brief Read baseline (10-bit, from 8-bit register << 2) for one electrode. */
mpr121_err_t mpr121_read_baseline(mpr121_t *dev, uint8_t ele, uint16_t *value);

/** @brief Update thresholds at run time (stop mode, write, run mode). */
mpr121_err_t mpr121_set_thresholds(mpr121_t *dev, const uint8_t *touch_th,
                                const uint8_t *release_th);

#endif /* MPR121_H */
