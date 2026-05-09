# Sumo Robot — Arduino Nano

Autonomous mini-sumo robot firmware for a 770 mm circular arena. Heavy-class
build (near the weight limit, modest top speed) — strategy is tuned for
mass-leveraged head-on rams rather than chase-and-circle play. Includes
banner discrimination so a swinging banner on the opponent doesn't pull our
front off the body.

## Hardware

| Subsystem        | Part               | Notes                                |
| ---------------- | ------------------ | ------------------------------------ |
| MCU              | Arduino Nano       | ATmega328P, 5 V                      |
| Distance sensors | 5× VL53L0X (I2C)   | XSHUT-based bring-up + readdressing  |
| Line sensors     | 2× analog reflectance | Black/white edge detection on A2/A7 |
| Motor driver     | A4950 dual H-bridge | Sign-magnitude PWM, slow-decay      |
| Drive            | 2× 300 RPM motors, 42 mm wheels | ~660 mm/s top speed       |
| Robot footprint  | 100 × 100 mm       | Near the class weight limit          |

### Arena assumptions
- Circular, 770 mm diameter (385 mm radius).
- 25 mm white outer ring; black surface inside.
- Robots placed opposite each other at start.

### Pin map

| Function                   | Pin  |
| -------------------------- | ---- |
| ToF Side Left  XSHUT       | D8   |
| ToF Front Left XSHUT       | D12  |
| ToF Front Center XSHUT     | D4   |
| ToF Front Right XSHUT      | D2   |
| ToF Side Right XSHUT       | D7   |
| Line sensor Right (analog) | A2   |
| Line sensor Left  (analog) | A7   |
| External start (digital)   | A0   |
| Local start button         | A1   |
| Kill signal (INT1, RISING) | D3   |
| Right motor IN1 (motor +)  | D6   |
| Right motor IN2 (motor −)  | D5   |
| Left motor IN1 (motor +)   | D10  |
| Left motor IN2 (motor −)   | D11  |

ToF sensors get re-assigned I2C addresses 0x30 (SL), 0x31 (FL), 0x32 (FC),
0x33 (FR), 0x34 (SR) at boot.

## Project layout

```
sumo/
├── sumo.ino       Setup, main loop, kill ISR, start sequence.
├── Config.h       Pin map, distance/timing/PWM tunables — edit here.
├── Sensors.h/.cpp VL53L0X bring-up + non-blocking continuous reads, line baseline calibration.
├── Motors.h/.cpp  A4950 sign-magnitude drive(left,right), brake/coast.
└── Strategy.h/.cpp Priority state machine (LINE_ESCAPE → ANTI_FLANK → RAM → TRACK → SEARCH).
```

## Strategy summary

Each loop iteration evaluates priorities top-down; first match owns the motors:

1. **LINE_ESCAPE** — white detected → reverse + turn away (latched maneuver).
   Both sides white → 180°. Single side → ~120° away from the edge.
2. **ANTI_FLANK** — banner-aware spin to face a real flank threat. Triggers
   only when the side reading is close (< 200 mm), meaningfully closer than
   every front reading (lead > 150 mm), and the predicate has held for at
   least 60 ms. Filters out swinging banners.
3. **RAM** — front center < 200 mm → full PWM forward. **Stall/collision
   boost**: once distance stops decreasing while close, latch full PWM
   through the push (sensor saturates on contact); release only on line trip
   or sustained distance recovery.
4. **TRACK** — FC-biased. If FC has any lock, drive straight unless FL or FR
   reads ≥ 150 mm closer (real angular offset). FL/FR alone are treated as
   suspect — possibly banner — and don't trigger ram.
5. **SEARCH** — spin in place toward `last_seen_side` (default left).

### Why these choices for a heavy/slow robot

- Mass advantage wins head-on rams. Stall boost keeps full PWM through
  contact instead of releasing as the ToF reading bounces.
- Spinning is expensive when you're slow — the FC-bias and persistence-
  gated anti-flank cut down on wasted turns chasing banner artefacts.
- FC-only ram trigger prevents charging at a banner edge that briefly enters
  FL/FR while the actual body is elsewhere.

## Build and flash

1. Install **Arduino IDE** (1.8 or 2.x).
2. **Library Manager** → install **VL53L0X by Pololu**.
3. Open `sumo/sumo.ino`.
4. Tools → Board: *Arduino Nano*. Processor: *ATmega328P* (or "Old Bootloader"
   if your Nano shipped with the older bootloader).
5. Select the COM port → **Upload**.

## Bring-up and tuning

Set `SUMO_DEBUG 1` in [`sumo/Config.h`](sumo/Config.h) for serial telemetry at 115200 baud.

| Step | What                                | Pass criterion                                 |
| ---- | ----------------------------------- | ---------------------------------------------- |
| 1    | ToF bring-up                        | Cover each sensor — only the named one drops below 100 mm. |
| 2    | Line sensors                        | White vs. black raw values differ by > 300 counts. |
| 3    | Motors (set `DEBUG_MOTOR_TEST 1`)   | Each wheel runs forward then reverse on cue. If reversed, swap that motor's `IN1`/`IN2` pin macros — don't rewire. |
| 4    | Start / kill                        | A0 high or A1 button starts the 5 s countdown; D3 high halts motors instantly. |
| 5    | Strategy dry run                    | Wave hands at sensors on blocks — state transitions print as expected. |
| 6    | Arena test                          | Push gently toward the white edge with no opponent — LINE_ESCAPE fires before wheels cross. |

In-arena calibration order:
1. `LINE_MARGIN` — first, so the safety net is solid.
2. `ENGAGE_DISTANCE` — when to commit to TRACK.
3. `RAM_DISTANCE` — when to go full power.
4. `FLANK_CLOSE_MM`, `FLANK_LEAD_MM`, `FC_BIAS_MM` — only after running a few rounds against a real banner robot.

## Tunables (in [`sumo/Config.h`](sumo/Config.h))

| Constant            | Default | Meaning                                            |
| ------------------- | ------- | -------------------------------------------------- |
| `ENGAGE_DISTANCE`   | 700 mm  | Inside this, TRACK takes over.                     |
| `RAM_DISTANCE`      | 200 mm  | Inside this on FC, RAM at full PWM.                |
| `FLANK_CLOSE_MM`    | 200 mm  | Min side closeness to consider a flank threat.     |
| `FLANK_LEAD_MM`     | 150 mm  | Side must be this much closer than every front.    |
| `FLANK_PERSIST_MS`  | 60 ms   | Predicate must hold this long before spin.         |
| `FC_BIAS_MM`        | 150 mm  | FL/FR must beat FC by this margin to override.     |
| `RAM_PWM`           | 255     | Full power on ram and stall-latch push.            |
| `FORWARD_PWM`       | 200     | Tracking forward speed.                            |
| `TURN_PWM`          | 200     | In-place spin speed for line escape and anti-flank.|
| `SEARCH_PWM`        | 160     | In-place spin speed during search.                 |
| `START_DELAY_MS`    | 5000    | Sumo-standard 5 s grace after start signal.        |
| `LINE_REVERSE_MS`   | 180     | Reverse duration after a line trip.                |
| `LINE_TURN_MS`      | 250     | ~120° spin after single-side line trip.            |
| `LINE_180_MS`       | 380     | ~180° spin after both-sides line trip.             |

## Safety notes

- The kill ISR (D3, RISING) directly drives all four motor pins LOW. It's
  attached after motor pin configuration in `setup()` so it's always valid.
- D3 is configured as plain `INPUT` — your kill source must drive it actively
  or include an external pull-down. If you'd rather use active-low, change
  the pin mode and the ISR mode (see [`sumo/sumo.ino`](sumo/sumo.ino)).
- The robot expects to be sitting on **black** at boot (line baseline
  calibration runs for ~200 ms during setup).

## License

TBD.
