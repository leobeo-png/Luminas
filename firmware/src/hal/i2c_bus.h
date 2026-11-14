/**
 * @file    i2c_bus.h
 * @brief   Bounded I2C master with timeout, bus recovery and error counters.
 *
 * @layer   hal
 * @core    1 (owns I2C0). I2C1 may be used from core 0 for the ToF experiment;
 *          one bus instance is never shared between cores.
 * @depends pico-sdk hardware_i2c, hardware_gpio (implementation only)
 * @timing  read_reg of 2 bytes at 400 kHz: ~115 us on the wire.
 *          Every call returns within timeout_us + ~20 us.
 *          i2c_bus_recover(): ~25 us at 400 kHz (9 clocks + STOP).
 * @errors  bus_err_t; every error increments a counter in i2c_bus_stats_t.
 * @isr     Not ISR-safe. Blocking, but bounded by timeout_us (SR-R-01).
 * @traces  SR-R-01, SR-R-02, SR-R-06, SR-E-06
 *
 * The header has no pico-sdk includes, so drivers can be unit-tested on the
 * host by linking a mock implementation (test/mock_i2c_bus.c) instead.
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Result of every bus operation. */
typedef enum
{
    BUS_OK = 0,     /**< Transfer completed, all bytes ACKed.               */
    BUS_TIMEOUT,    /**< Transfer did not complete within timeout_us.       */
    BUS_NACK,       /**< Address or data byte not acknowledged.             */
    BUS_STUCK,      /**< SDA or SCL held low; recovery failed.              */
    BUS_ARG         /**< Invalid argument (NULL, len 0, bus not initialised).*/
} bus_err_t;

/** Per-bus error counters, readable over CDC (SR-R-06). */
typedef struct
{
    uint32_t transfers;     /**< Total transfers attempted.                 */
    uint32_t timeouts;
    uint32_t nacks;
    uint32_t recoveries;    /**< Successful i2c_bus_recover() calls.        */
    uint32_t stuck;         /**< Recoveries that failed (BUS_STUCK).        */
} i2c_bus_stats_t;

/** Bus instance. Allocate statically; treat fields as private. */
typedef struct
{
    uint8_t         instance;   /**< 0 = I2C0, 1 = I2C1.                    */
    uint8_t         pin_sda;
    uint8_t         pin_scl;
    uint32_t        baud_hz;
    bool            ready;
    i2c_bus_stats_t stats;
} i2c_bus_t;

/**
 * @brief  Initialise the peripheral and pins. Does not enable internal
 *         pull-ups (external pull-ups on the board, SR-E-06).
 * @pre    bus != NULL; pins valid for the instance (RP2040 §2.19.2).
 * @return BUS_OK, or BUS_STUCK if SDA/SCL read low after init and
 *         recovery failed, or BUS_ARG.
 */
bus_err_t i2c_bus_init(i2c_bus_t *bus, uint8_t instance,
                       uint8_t pin_sda, uint8_t pin_scl, uint32_t baud_hz);

/**
 * @brief  Write register address, repeated START, read len bytes.
 * @pre    bus ready; buf != NULL; 1 <= len <= 64.
 * @post   On error, buf contents are undefined; caller must not use them
 *         (old bug: 0xFF bytes read as "all keys pressed").
 */
bus_err_t i2c_bus_read_reg(i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                           uint8_t *buf, size_t len, uint32_t timeout_us);

/**
 * @brief  Write register address followed by len data bytes, then STOP.
 * @pre    bus ready; buf != NULL; 1 <= len <= 64.
 */
bus_err_t i2c_bus_write_reg(i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                            const uint8_t *buf, size_t len,
                            uint32_t timeout_us);

/**
 * @brief  Bus recovery (SR-R-02): switch pins to GPIO, clock SCL up to 9
 *         times until SDA is released, generate STOP, re-init peripheral.
 * @return BUS_OK if both lines read high afterwards, else BUS_STUCK.
 * @note   Devices on the bus must be re-initialised by their drivers.
 */
bus_err_t i2c_bus_recover(i2c_bus_t *bus);

/** @brief Probe an address with a 1-byte read. BUS_OK if ACKed. */
bus_err_t i2c_bus_probe(i2c_bus_t *bus, uint8_t addr, uint32_t timeout_us);

/** @brief Copy the counters. Safe to call from the other core (torn reads
 *         of individual counters are acceptable for diagnostics). */
void i2c_bus_get_stats(const i2c_bus_t *bus, i2c_bus_stats_t *out);

/** @brief Zero the counters. */
void i2c_bus_reset_stats(i2c_bus_t *bus);

#endif /* I2C_BUS_H */
