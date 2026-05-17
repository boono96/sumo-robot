# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Arduino Nano firmware for an autonomous mini-sumo robot. Single sketch under `sumo/` — no host-side code, no test harness, no build scripts. Target: ATmega328P, 5 V. The robot is heavy/slow and the strategy is explicitly tuned for that profile (mass-leveraged head-on rams). Opponents are assumed to carry a static side-extending banner, treated as part of a wide-front target — no banner-discrimination logic.

## Build / flash

There is no CLI build. Use the Arduino IDE (1.8 or 2.x):

1. Library Manager → install **VL53L0X by Pololu** (required — `Sensors.cpp` includes `<VL53L0X.h>`).
2. Open [`sumo/sumo.ino`](sumo/sumo.ino).
3. Tools → Board: *Arduino Nano*; Processor: *ATmega328P* (try "Old Bootloader" if upload fails).
4. Select COM port → Upload.

No automated tests exist. Behavior changes are validated by serial telemetry and arena runs — see the bring-up table in [README.md](README.md).

## Architecture

The sketch is split into four translation units in `sumo/`. Each `.cpp` owns its module's persistent state inside an anonymous namespace; cross-module communication is by value-type structs only (`ToFReadings`, `LineReadings`) — no globals are shared across modules.

- [sumo.ino](sumo/sumo.ino) — `setup()`/`loop()`, run-mode FSM (`MODE_IDLE → MODE_ARMED → MODE_RUNNING → MODE_KILLED`), start-trigger / countdown logic, and the kill ISR. The kill ISR drives all four motor pins LOW directly and is attached **after** `Motors::begin()` so the writes are always valid.
- [Config.h](sumo/Config.h) — single source of truth for **all** pin assignments, distance thresholds, PWM levels, timings, and debug flags. Tuning happens here; module code reads these constants directly. Three debug macros gate optional code: `SUMO_DEBUG` (loop telemetry), `DEBUG_MOTOR_TEST` (twitch test in setup), `VL53L0X_DEBUG` (ToF bring-up tracing + stuck-sensor LED blink codes).
- [Sensors.{h,cpp}](sumo/Sensors.cpp) — VL53L0X bring-up via XSHUT sequencing with re-assigned I2C addresses (0x30–0x34), continuous-mode reads clamped to `TOF_NO_TARGET`, and fixed-threshold analog line reads. **No boot-time line calibration** — thresholds are hard-coded in `Config.h` and must be set per-arena.
- [Motors.{h,cpp}](sumo/Motors.cpp) — A4950 sign-magnitude drive. `drive(left, right)` with values in [-255, +255]; `drive(0, 0)` is **brake** (both pins HIGH), not coast. Use `coast()` explicitly (e.g. on kill).
- [Strategy.{h,cpp}](sumo/Strategy.cpp) — priority-ordered state machine, evaluated top-down each `loop()`; first match owns the motors and returns:

  1. `LINE_ESCAPE` (latched REVERSE → TURN, preempts everything; both-sides-white triggers a 180°, single-side triggers ~120° away)
  2. `RAM` (FC < `RAM_DISTANCE`, with a stall/collision latch that holds full PWM through contact even when the ToF reading bounces — release only on sustained distance recovery or line trip)
  3. `TRACK` (FC-biased: FL/FR must beat FC by `FC_BIAS_MM` to override straight-ahead drive; FC-blind fallback curves toward whichever of FL/FR reads closer)
  4. `SEARCH` (in-place spin toward `last_seen_side`, default left; SL/SR readings inside `ENGAGE_DISTANCE` bias `last_seen_side` so search heads toward the last detection)

  Each latched behavior keeps its end-time and direction in file-static state and clears it on completion. Adding a new state means: add an enum, slot it into `Strategy::step()` at the right priority, and reset any new state in `Strategy::reset()`.

### Cross-cutting invariants

- **Tuning lives in [Config.h](sumo/Config.h).** Don't introduce magic numbers in `.cpp` files — add a `constexpr` to `Config.h` and reference it.
- **Opponent banner is treated as wide-front target, not noise.** Side and FL/FR hits are valid signals; do not reintroduce banner-discrimination or anti-flank logic without checking with the user first — it was explicitly removed.
- **Kill safety:** any new motor writes must remain reachable by the ISR's all-pins-LOW behavior, i.e. don't add motor outputs on pins outside the four `PIN_*_IN{1,2}` macros without also extending `killISR()` and `Motors::coast()`.
- **Pin reassignments:** if a motor wire is flipped, change the `PIN_*_IN1`/`PIN_*_IN2` macros in `Config.h` rather than rewiring (per the bring-up notes).

## Notes for future edits

- `START_DELAY_MS` in [Config.h](sumo/Config.h) currently reads `10` (effectively no grace period) while the README documents "sumo standard 5 s" — likely a debug leftover. Confirm with the user before changing.
- `Serial.begin()` baud differs between `sumo.ino` (9600) and the README (says 115200). The code is the source of truth (9600).
