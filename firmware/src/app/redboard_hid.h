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
 * @traces  SR-I-01, SR-I-02, SR-I-03, SR-I-06, SR-I-07, SR-P-03
 *
 * Reviewed against RedBoard chuniio.c (8 Nov 2026):
 *   - Input report: 9 bytes packed {id, u16 buttons, u8 HAT, u32 axis,
 *     u8 vendor}. DLL reads it with hid_get_report() -> answer GET_REPORT
 *     on the control pipe (tud_hid_get_report_cb) AND send it on the
 *     interrupt IN endpoint every frame. HAT and vendor are ignored by the
 *     DLL (send 0x08 / 0x00).
 *   - Slider: DLL does axis ^ 0x80808080, bit i -> cell i (segatools cell
 *     order; direction to verify on hardware).
 *   - Air bits are NOT in order: game beam 1 = report bit 3, beams 2..4 =
 *     bits 0..2, beams 5..6 = bits 4..5. Use redboard_air_to_buttons().
 *   - Operator buttons: DLL maps START (0x200) -> segatools TEST and
 *     SERVICE (0x100) or TEST (0x1000) -> segatools SERVICE. No coin over
 *     HID: coin is keyboard-only in this DLL (vk_coin, F3).
 *     Luminas has no operator buttons (D13): bits 0x100/0x200/0x1000 are
 *     always 0; Test/Service/Coin come from the PC keyboard.
 *   - LEDs on interface 1 (game input must be interface 0):
 *       report 4 (output): 48 bytes = brg[0..47]
 *       report 5 (output): 48 bytes = brg[48..92] + 3 padding bytes
 *       report 6 (output): 18 bytes = 6 tower LEDs BRG (Luminas has none:
 *                          accept and ignore)
 *       report 0x0B (feature, 64 bytes incl. ID): [1] = LZFX length,
 *                          [2..] = LZFX data of the 93-byte frame.
 *     DLL quirk: it allows a compressed length up to 63 but sends only 64
 *     bytes incl. ID and length (62 data bytes max). Reject length > 62.
 *   - LZFX decompressor needed in firmware (BSD-2; attribute, SR-C-05).
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

#define REDBOARD_INTERFACE_INPUT    (0U)
#define REDBOARD_INTERFACE_LIGHTS   (1U)

#define REDBOARD_LED_A_LEN          (48U)
#define REDBOARD_LED_B_LEN          (48U)   /* 45 used */
#define REDBOARD_LED_B_USED         (45U)
#define REDBOARD_AIR_LED_LEN        (18U)
#define REDBOARD_LZFX_REPORT_LEN    (63U)   /* excl. report ID */
#define REDBOARD_LZFX_MAX_DATA      (62U)

/* Button bits (SR-F-10); air beams occupy bits 0..5 (SR-I-02).
 * Names are RedBoard's; effect in segatools via this DLL in brackets. */
#define REDBOARD_BTN_AIR_MASK       (0x003FU)
#define REDBOARD_BTN_SERVICE        (0x0100U)   /* -> SERVICE */
#define REDBOARD_BTN_START          (0x0200U)   /* -> TEST    */
#define REDBOARD_BTN_TEST           (0x1000U)   /* -> SERVICE */
#define REDBOARD_HAT_NEUTRAL        (0x08U)

#define REDBOARD_SLIDER_XOR         (0x80808080UL)

/** Input report, report ID 1. Packed; little-endian on the wire. */
typedef struct __attribute__((packed))
{
    uint8_t  report_id;     /**< REDBOARD_REPORT_ID_INPUT              */
    uint16_t buttons;       /**< air bits 0..5 + TEST/SERVICE/START    */
    uint8_t  hat;           /**< REDBOARD_HAT_NEUTRAL (ignored by DLL) */
    uint32_t slider;        /**< 32-zone bitmap XOR REDBOARD_SLIDER_XOR*/
    uint8_t  vendor;        /**< 0x00 (ignored by DLL)                 */
} redboard_input_report_t;

_Static_assert(sizeof(redboard_input_report_t) == 9U,
               "RedBoard input report must be 9 bytes");

/**
 * @brief  Reorder air_read() bits (bit n = beam n+1, bottom first) into
 *         RedBoard button bits: beam1 -> bit3, beams2..4 -> bits0..2,
 *         beams5..6 -> bits4..5. Inverse of the DLL's conversion.
 */
uint16_t redboard_air_to_buttons(uint8_t air_mask);

/**
 * @brief  Build the input report from current state.
 * @param  slider_mask  touch_mask(), bit n = zone n
 * @param  air_mask     air_read(), 6 bits
 */
void redboard_hid_build(redboard_input_report_t *r, uint32_t slider_mask,
                        uint8_t air_mask);

/**
 * @brief  Handle an output/feature report on interface 1 (lights).
 *         Assembles the 93-byte BRG frame and passes it to leds_set_brg().
 *         Report 6 accepted and ignored. LZFX: decompressed length must be
 *         exactly 93, else rejected.
 * @return true if the report was recognised and well-formed.
 */
bool redboard_hid_on_led_report(uint8_t report_id, const uint8_t *buf,
                                size_t len, uint32_t now_us);

#endif /* REDBOARD_HID_H */
