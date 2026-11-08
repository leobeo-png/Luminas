/**
 * @file    redboard_hid.h
 * @brief   RedBoard-compatible HID reports (RedBoard commit 597e5a3,
 *          chuniio.c): input report build, LED output report parsing.
 *
 * @layer   app (report build/parse is pure and host-tested; the TinyUSB
 *          callbacks live in usb_descriptors.c / main_core0.c)
 * @core    0
 * @depends svc/leds.h (LED data hand-off)
 * @timing  redboard_hid_build(): < 5 us.
 * @errors  Parse returns false on unknown ID or wrong length.
 * @isr     Not ISR-safe.
 * @traces  SR-I-01, SR-I-02, SR-I-03, SR-I-06, SR-I-07, SR-F-10, SR-P-03
 *
 * TO VERIFY against chuniio.c before implementation: exact byte order of
 * the input report, LED report 4/5 split of the 93 bytes, LZFX report 0x0B
 * size, air-tower report 6 length.
 */
#ifndef REDBOARD_HID_H
#define REDBOARD_HID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REDBOARD_VID                (0x0F0DU)
#define REDBOARD_PID                (0x0092U)

#define REDBOARD_REPORT_ID_INPUT    (1U)
#define REDBOARD_REPORT_ID_LED_A    (4U)
#define REDBOARD_REPORT_ID_LED_B    (5U)
#define REDBOARD_REPORT_ID_AIR_LED  (6U)
#define REDBOARD_REPORT_ID_LED_LZFX (0x0BU)

/* Button bits (SR-F-10); air beams occupy bits 0..5 (SR-I-02) */
#define REDBOARD_BTN_AIR_MASK       (0x003FU)
#define REDBOARD_BTN_SERVICE        (0x0100U)
#define REDBOARD_BTN_START          (0x0200U)
#define REDBOARD_BTN_TEST           (0x1000U)

#define REDBOARD_SLIDER_XOR         (0x80808080UL)

/** Input report, report ID 1. Packed; little-endian on the wire. */
typedef struct __attribute__((packed))
{
    uint8_t  report_id;     /**< REDBOARD_REPORT_ID_INPUT              */
    uint16_t buttons;       /**< air bits 0..5 + TEST/SERVICE/START    */
    uint8_t  hat;           /**< HAT byte (neutral value: verify)      */
    uint32_t slider;        /**< 32-zone bitmap XOR REDBOARD_SLIDER_XOR*/
    uint8_t  vendor;        /**< vendor byte (verify value)            */
} redboard_input_report_t;

/**
 * @brief  Build the input report from current state.
 * @param  slider_mask  touch_mask(), bit n = zone n
 * @param  air_mask     air_read(), 6 bits
 * @param  buttons      REDBOARD_BTN_* bits for TEST/SERVICE/START
 */
void redboard_hid_build(redboard_input_report_t *r, uint32_t slider_mask,
                        uint8_t air_mask, uint16_t buttons);

/**
 * @brief  Handle an output/feature report on interface 1 (lights).
 *         Assembles the 93-byte BRG frame and passes it to leds_set_brg().
 * @return true if the report was recognised and well-formed.
 */
bool redboard_hid_on_led_report(uint8_t report_id, const uint8_t *buf,
                                size_t len, uint32_t now_us);

#endif /* REDBOARD_HID_H */
