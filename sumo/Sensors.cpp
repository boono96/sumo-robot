#include "Sensors.h"
#include "Config.h"
#include <Wire.h>
#include <VL53L0X.h>

namespace {
  VL53L0X tof_sl, tof_fl, tof_fc, tof_fr, tof_sr;

  int line_l_threshold = 600;
  int line_r_threshold = 600;

  void bringUp(VL53L0X& s, uint8_t xshut_pin, uint8_t new_addr) {
    pinMode(xshut_pin, INPUT);   // release reset; sensor wakes via internal pull-up
    delay(10);
    s.setTimeout(50);
    if (!s.init()) {
      // Stuck low-level fault — flash onboard LED forever.
      pinMode(LED_BUILTIN, OUTPUT);
      while (true) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
        digitalWrite(LED_BUILTIN, LOW);  delay(100);
      }
    }
    s.setAddress(new_addr);
    s.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    s.startContinuous(0);
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
  // Hold all five in reset.
  pinMode(PIN_XSHUT_SL, OUTPUT); digitalWrite(PIN_XSHUT_SL, LOW);
  pinMode(PIN_XSHUT_FL, OUTPUT); digitalWrite(PIN_XSHUT_FL, LOW);
  pinMode(PIN_XSHUT_FC, OUTPUT); digitalWrite(PIN_XSHUT_FC, LOW);
  pinMode(PIN_XSHUT_FR, OUTPUT); digitalWrite(PIN_XSHUT_FR, LOW);
  pinMode(PIN_XSHUT_SR, OUTPUT); digitalWrite(PIN_XSHUT_SR, LOW);
  delay(10);

  bringUp(tof_sl, PIN_XSHUT_SL, TOF_ADDR_SL);
  bringUp(tof_fl, PIN_XSHUT_FL, TOF_ADDR_FL);
  bringUp(tof_fc, PIN_XSHUT_FC, TOF_ADDR_FC);
  bringUp(tof_fr, PIN_XSHUT_FR, TOF_ADDR_FR);
  bringUp(tof_sr, PIN_XSHUT_SR, TOF_ADDR_SR);
}

void Sensors::calibrateLineBaseline() {
  uint32_t sum_l = 0, sum_r = 0;
  uint16_t n = 0;
  uint32_t end = millis() + LINE_CAL_MS;
  while ((int32_t)(end - millis()) > 0) {
    sum_l += analogRead(PIN_LINE_L);
    sum_r += analogRead(PIN_LINE_R);
    n++;
    delay(2);
  }
  if (n == 0) n = 1;
  line_l_threshold = (sum_l / n) + LINE_MARGIN;
  line_r_threshold = (sum_r / n) + LINE_MARGIN;
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
  r.left_white  = r.left_raw  > line_l_threshold;
  r.right_white = r.right_raw > line_r_threshold;
  return r;
}
