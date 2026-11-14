/**
 * @file    cli.h
 * @brief   Line-based command interpreter on the CDC port (SR-F-08).
 *
 * @layer   app (parser is pure and host-tested; I/O through callbacks)
 * @core    0
 * @depends svc/touch, svc/air, svc/leds, svc/calib, hal/flash_store,
 *          hal/i2c_bus (stats), drv/mpr121 (raw/baseline via core 1 snapshot)
 * @timing  cli_feed(): O(1) per char; command execution < 1 ms except
 *          "save" (flash erase, see flash_store.h).
 * @errors  Unknown/malformed commands print "ERR <reason>".
 * @isr     Not ISR-safe.
 * @traces  SR-F-08, SR-F-09, SR-R-04, SR-R-06, SR-E-02, UR-07
 *
 * Commands (draft):
 *   help
 *   status              reset cause, uptime, core1 heartbeat rate
 *   touch               32-zone mask + per-chip online
 *   raw <chip>          filtered + baseline per electrode
 *   th <chip> <ele> <touch> <release>
 *   air                 OFF/ON/signal/saturated per beam
 *   airth <beam> <counts> | airhyst <counts>
 *   bright <pct>        clamped 5..cap
 *   i2c                 bus counters (timeouts, NACKs, recoveries)
 *   save | load | defaults
 *
 * Must never block HID: output is buffered and dropped if the host is not
 * reading (CDC closed), SR-I-06.
 */
#ifndef CLI_H
#define CLI_H

#include <stddef.h>
#include <stdint.h>

/** Output sink; returns bytes accepted (may be fewer than len). */
typedef size_t (*cli_write_fn_t)(const char *buf, size_t len);

void cli_init(cli_write_fn_t write);

/** @brief Feed received bytes; executes on '\r' or '\n'. Max line 64. */
void cli_feed(const char *buf, size_t len);

#endif /* CLI_H */
