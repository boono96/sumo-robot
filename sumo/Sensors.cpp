#include "Sensors.h"
#include "Config.h"
#include <Wire.h>
#include <VL53L0X.h>

namespace {
  VL53L0X tof_sl, tof_fl, tof_fc, tof_fr, tof_sr;

  // Stuck-sensor indicator: flash `id` short pulses, long pause, repeat forever.
  // id = 1..5 for SL, FL, FC, FR, SR.
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

  void bringUp(VL53L0X& s, uint8_t xshut_pin, uint8_t new_addr,
               uint8_t id, const __FlashStringHelper* name) {
#if VL53L0X_DEBUG
    Serial.print(F("[ToF] release XSHUT ")); Serial.println(name);
#endif
    pinMode(xshut_pin, INPUT);   // release reset; sensor wakes via internal pull-up
    delay(10);

#if VL53L0X_DEBUG
    Serial.print(F("[ToF] init ")); Serial.println(name);
#endif
    s.setTimeout(50);
    if (!s.init()) {
#if VL53L0X_DEBUG
      Serial.print(F("[ToF] STUCK at init: ")); Serial.println(name);
      Serial.flush();
#endif
      stuckBlink(id);
    }

#if VL53L0X_DEBUG
    Serial.print(F("[ToF] setAddress 0x")); Serial.println(new_addr, HEX);
#endif
    s.setAddress(new_addr);
    s.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    s.startContinuous(0);
#if VL53L0X_DEBUG
    Serial.print(F("[ToF] OK ")); Serial.println(name);
#endif
  }

  uint16_t readOne(VL53L0X& s) {
    uint16_t mm = s.readRangeContinuousMillimeters();
    if (s.timeoutOccurred() || mm == 65535 || mm > TOF_NO_TARGET) {
      return TOF_NO_TARGET;
    }
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

  bringUp(tof_sl, PIN_XSHUT_SL, TOF_ADDR_SL, 1, F("SL"));
  bringUp(tof_fl, PIN_XSHUT_FL, TOF_ADDR_FL, 2, F("FL"));
  bringUp(tof_fc, PIN_XSHUT_FC, TOF_ADDR_FC, 3, F("FC"));
  bringUp(tof_fr, PIN_XSHUT_FR, TOF_ADDR_FR, 4, F("FR"));
  bringUp(tof_sr, PIN_XSHUT_SR, TOF_ADDR_SR, 5, F("SR"));

#if VL53L0X_DEBUG
  Serial.println(F("[ToF] all five up"));
#endif
}

ToFReadings Sensors::readToF() {
  ToFReadings r;
  r.sl = readOne(tof_sl);
  r.fl = readOne(tof_fl);
  r.fc = readOne(tof_fc);
  r.fr = readOne(tof_fr);
  r.sr = readOne(tof_sr);
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
