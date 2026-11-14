/**
 * @file    luminas_pins.h
 * @brief   Luminas main board pin map (Architecture v0.4, section 3).
 *
 * @layer   board (no code, constants only; host-compilable)
 * @traces  SR-I-08, SR-R-07
 *
 * Module: Raspberry Pi Pico-compatible USB-C clone (D4).
 * GP23, GP24 and GP29 are module-internal and must not be used.
 */
#ifndef LUMINAS_PINS_H
#define LUMINAS_PINS_H

/* Debug UART (UART0) */
#define PIN_UART_TX         (0U)
#define PIN_UART_RX         (1U)

/* I2C1: spare header (ToF experiment) */
#define PIN_I2C1_SDA        (2U)
#define PIN_I2C1_SCL        (3U)

/* I2C0: MPR121 x3 (0x5A, 0x5B, 0x5C) */
#define PIN_I2C0_SDA        (4U)
#define PIN_I2C0_SCL        (5U)

/* MPR121 IRQ lines: open-drain, active low, 10 kOhm external pull-up */
#define PIN_TOUCH_IRQ0      (6U)
#define PIN_TOUCH_IRQ1      (7U)
#define PIN_TOUCH_IRQ2      (8U)

/* IR air towers (Appendix A) */
#define PIN_IR_SEL0         (10U)
#define PIN_IR_SEL1         (11U)
#define PIN_IR_SEL2         (12U)
#define PIN_IR_EN           (13U)   /* 100 kOhm external pull-down (R25)   */

/* Power */
#define PIN_LED_5V_EN       (14U)   /* D12 Q2 gate, 100 kOhm pull-down (R3) */

/* WS2812B data, PIO0 SM0, via 74AHCT1G125 */
#define PIN_LED_DATA        (15U)

/* Debug markers for latency measurement */
#define PIN_DBG0            (19U)
#define PIN_DBG1            (20U)
#define PIN_DBG2            (21U)

/* Module onboard LED */
#define PIN_STATUS_LED      (25U)

/* ADC */
#define PIN_AIR_OUT         (26U)   /* ADC0 */
#define PIN_V5SW_SENSE      (27U)   /* ADC1, optional divider */
#define ADC_CH_AIR_OUT      (0U)
#define ADC_CH_V5SW_SENSE   (1U)

/* Spare, broken out to pads: GP9, GP16, GP17, GP18, GP22, GP28 (ADC2).
 * No operator buttons: Test/Service/Coin stay on the PC keyboard (D13). */

/* I2C addresses (ADDR straps) */
#define MPR121_ADDR_0       (0x5AU)
#define MPR121_ADDR_1       (0x5BU)
#define MPR121_ADDR_2       (0x5CU)

#endif /* LUMINAS_PINS_H */
