# Luminas firmware (RP2040, pico-sdk + TinyUSB)

Status: **Phase 1 draft — module headers only, for review.** No implementation yet.

Source docs: "Luminas Architecture - Phase 1" v0.4 (§7 software architecture, §3 pin map), "Luminas Requirements - Phase 0".

## Layout

```
src/
  board/  luminas_pins.h        pin map + I2C addresses
  hal/    i2c_bus.h             bounded I2C, recovery, counters
          ws2812_pio.h          PIO0 + DMA LED output
          adc_air.h             ADC0/ADC1 single-shot reads
          gpio_dbg.h            DBG0..2 latency markers
          flash_store.h         calibration sector (NEW, see below)
  drv/    mpr121.h              touch controller driver
          ir_air.h              IR beam scanner (timer state machine)
  svc/    touch.h               zone mapping, debounce, chip validity
          air.h                 beam decision, hysteresis, saturation
          leds.h                frame build, brightness, current limiter
          calib.h               calibration record, CRC
  app/    redboard_hid.h        RedBoard HID input/LED reports
          cli.h                 CDC command line
          core_shared.h         inter-core data, watchdog heartbeat
```

Rule: a layer calls only the layer below it. `svc/` and `app/` logic has no pico-sdk includes and is unit-tested on the host (Unity + ctest). Driver headers include only `hal/` headers, so drivers are tested against mocks (`test/mock_i2c_bus.c`, later).

Every header starts with the same contract block: `@layer @core @depends @timing @errors @isr @traces`.

## Review points (changes vs Architecture v0.4)

1. **Thresholding moved from `drv/ir_air` to `svc/air`.** The driver only produces raw OFF/ON samples; `air_read()` and `air_saturated()` live in `svc/air.h`. Reason: decision logic becomes host-testable.
2. **New `hal/flash_store`** split out of `svc/calib`, so `calib` stays pure.
3. **New `app/core_shared.h`**: live masks (lock-free) and diagnostics (seqlock).
4. **MPR121 does touch/release hysteresis in hardware**; `svc/touch` only debounces, maps electrodes to zones and tracks chip validity.
5. **`mpr121.h` has its own error enum** (`mpr121_err_t`, incl. over-current flag OVCF); the bus error is kept in `dev->last_bus_err`.
6. **Open in `redboard_hid.h`**: verify the input report byte order, the LED report 4/5 split, and the LZFX/air-tower report lengths against chuniio.c (commit 597e5a3) before implementation.
7. **Electrode → zone table** in `touch.c` to be fixed with the schematic (32 of 36 inputs used).

Host syntax check: `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only -Isrc src/*/*.h`.
