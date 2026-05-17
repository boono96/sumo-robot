#include "Sensors.h"
#include "Config.h"
#include <Wire.h>
#include <VL53L0X.h>

namespace {
  // ---- Per-sensor state ----
  struct ToFEntry {
    VL53L0X*  s;
    uint8_t   xshut_pin;
    uint8_t   addr;
    uint8_t   id;                          // 1..5, used for stuckBlink
    const __FlashStringHelper* name;
    uint8_t   bad_count;                   // consecutive timeouts / 65535 reads
  };

  VL53L0X tof_sl, tof_fl, tof_fc, tof_fr, tof_sr;

  ToFEntry tofs[5] = {
    { &tof_sl, PIN_XSHUT_SL, TOF_ADDR_SL, 1, F("SL"), 0 },
    { &tof_fl, PIN_XSHUT_FL, TOF_ADDR_FL, 2, F("FL"), 0 },
    { &tof_fc, PIN_XSHUT_FC, TOF_ADDR_FC, 3, F("FC"), 0 },
    { &tof_fr, PIN_XSHUT_FR, TOF_ADDR_FR, 4, F("FR"), 0 },
    { &tof_sr, PIN_XSHUT_SR, TOF_ADDR_SR, 5, F("SR"), 0 },
  };

  // After this many consecutive bad reads in a row, force-recover that sensor.
  // 5 * ~50 ms timeout = ~250 ms worst-case latency before recovery kicks in,
  // which is short enough not to wreck strategy timing.
  constexpr uint8_t TOF_BAD_THRESHOLD = 5;

  // Stuck-sensor indicator at boot: flash `id` short pulses, long pause, repeat.
  // Only used during initToF() — not during runtime recovery.
  void stuckBlink(uint8_t id) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      for (uint8_t i = 0; i < id; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(150);
        digitalWrite(LED_BUILTIN, LOW);  delay(200);
      }
      delay(1200);
    }
  }

  // Configure one sensor: release XSHUT, init at default 0x29, move to its
  // reassigned address, start continuous ranging. Returns true on success.
  bool configure(ToFEntry& e) {
    e.s->setTimeout(50);
    if (!e.s->init()) return false;
    e.s->setAddress(e.addr);
    e.s->setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    e.s->startContinuous(0);
    return true;
  }

  // Boot-time bring-up. Hangs on stuckBlink if a sensor doesn't come up — the
  // robot can't run blind anyway, and we want the failure to be visible.
  void bringUp(ToFEntry& e) {
#if VL53L0X_DEBUG
    Serial.print(F("[ToF] release XSHUT ")); Serial.println(e.name);
#endif
    pinMode(e.xshut_pin, INPUT);   // breakout has internal pull-up
    delay(10);

#if VL53L0X_DEBUG
    Serial.print(F("[ToF] init ")); Serial.println(e.name);
#endif
    if (!configure(e)) {
#if VL53L0X_DEBUG
      Serial.print(F("[ToF] STUCK at init: ")); Serial.println(e.name);
      Serial.flush();
#endif
      stuckBlink(e.id);
    }
#if VL53L0X_DEBUG
    Serial.print(F("[ToF] OK ")); Serial.println(e.name);
#endif
  }

  // Runtime recovery for one sensor: full XSHUT-cycle reset, re-init, restore
  // its custom address, restart continuous mode. Other sensors are unaffected
  // because they're already at addresses 0x30-0x34 — only this one comes back
  // up at 0x29 and is immediately moved to its assigned address.
  // Best-effort: if init() fails (e.g. bus still glitched), just return and we
  // will retry on the next threshold hit. Does NOT call stuckBlink — we never
  // want to deadlock mid-match.
  void recoverSensor(ToFEntry& e) {
#if VL53L0X_DEBUG
    Serial.print(F("[ToF] recover ")); Serial.println(e.name);
#endif
    pinMode(e.xshut_pin, OUTPUT);
    digitalWrite(e.xshut_pin, LOW);
    delay(5);
    pinMode(e.xshut_pin, INPUT);   // release, internal pull-up wakes sensor
    delay(10);

    if (!configure(e)) {
#if VL53L0X_DEBUG
      Serial.print(F("[ToF] recover FAILED ")); Serial.println(e.name);
#endif
      return;
    }
#if VL53L0X_DEBUG
    Serial.print(F("[ToF] recover OK ")); Serial.println(e.name);
#endif
  }

  uint16_t readOne(ToFEntry& e) {
    uint16_t mm = e.s->readRangeContinuousMillimeters();
    // The Pololu library returns 65535 on timeout. timeoutOccurred() also
    // clears the internal flag, so call it exactly once per read.
    bool bad = e.s->timeoutOccurred() || mm == 65535;
    if (bad) {
      if (++e.bad_count >= TOF_BAD_THRESHOLD) {
        recoverSensor(e);
        e.bad_count = 0;
      }
      return TOF_NO_TARGET;
    }
    e.bad_count = 0;
    if (mm > TOF_NO_TARGET) return TOF_NO_TARGET;
    return mm;
  }
}

void Sensors::initToF() {
#if VL53L0X_DEBUG
  // Make sure Serial is up so trace lines aren't lost. Safe to call even if
  // SUMO_DEBUG already opened it.
  if (!Serial) Serial.begin(9600);
  Serial.println(F("[ToF] initToF() begin"));
#endif

  // Hold all five in reset.
  pinMode(PIN_XSHUT_SL, OUTPUT); digitalWrite(PIN_XSHUT_SL, LOW);
  pinMode(PIN_XSHUT_FL, OUTPUT); digitalWrite(PIN_XSHUT_FL, LOW);
  pinMode(PIN_XSHUT_FC, OUTPUT); digitalWrite(PIN_XSHUT_FC, LOW);
  pinMode(PIN_XSHUT_FR, OUTPUT); digitalWrite(PIN_XSHUT_FR, LOW);
  pinMode(PIN_XSHUT_SR, OUTPUT); digitalWrite(PIN_XSHUT_SR, LOW);
  delay(10);

  for (uint8_t i = 0; i < 5; i++) bringUp(tofs[i]);

#if VL53L0X_DEBUG
  Serial.println(F("[ToF] all five up"));
#endif
}

ToFReadings Sensors::readToF() {
  ToFReadings r;
  r.sl = readOne(tofs[0]);
  r.fl = readOne(tofs[1]);
  r.fc = readOne(tofs[2]);
  r.fr = readOne(tofs[3]);
  r.sr = readOne(tofs[4]);
  return r;
}

LineReadings Sensors::readLine() {
  LineReadings r;
  r.left_raw  = analogRead(PIN_LINE_L);
  r.right_raw = analogRead(PIN_LINE_R);
  r.left_white  = r.left_raw  < LINE_L_THRESHOLD;
  r.right_white = r.right_raw < LINE_R_THRESHOLD;
  return r;
}
