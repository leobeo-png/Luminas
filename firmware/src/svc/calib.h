/**
 * @file    calib.h
 * @brief   Calibration record: defaults, validation, (de)serialisation.
 *
 * @layer   svc (pure logic; host unit-tested). Flash access is in
 *          hal/flash_store.h and is called by app/cli.
 * @core    0
 * @depends none
 * @timing  calib_check(): CRC32 over < 256 bytes, < 50 us.
 * @errors  calib_err_t.
 * @isr     Not ISR-safe.
 * @traces  SR-F-09, SR-F-03, SR-E-02
 */
#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>

#define CALIB_MAGIC     (0x4C554D31UL)  /* "LUM1" */
#define CALIB_VERSION   (1U)

typedef enum
{
    CALIB_OK = 0,
    CALIB_EMPTY,    /**< Erased flash (magic 0xFFFFFFFF).        */
    CALIB_MAGIC_BAD,
    CALIB_VERSION_BAD,
    CALIB_CRC_BAD
} calib_err_t;

/** Stored record. Size must stay <= FLASH_STORE_MAX_LEN (256). */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;                  /**< sizeof(calib_data_t).          */
    uint8_t  touch_th[3][12];       /**< MPR121 touch thresholds.       */
    uint8_t  release_th[3][12];     /**< MPR121 release thresholds.     */
    uint16_t air_block_th[6];       /**< ADC counts.                    */
    uint16_t air_hyst;
    uint8_t  led_brightness_pct;
    uint8_t  reserved[3];
    uint32_t crc32;                 /**< Over all bytes before crc32.   */
} calib_data_t;

/** @brief Fill with defaults (used when flash is empty or invalid). */
void calib_defaults(calib_data_t *c);

/** @brief Set magic/version/size and compute crc32 before saving. */
void calib_seal(calib_data_t *c);

/** @brief Validate a record read from flash. */
calib_err_t calib_check(const calib_data_t *c);

/** @brief CRC-32 (IEEE 802.3), exposed for unit tests. */
uint32_t calib_crc32(const void *data, uint32_t len);

#endif /* CALIB_H */
