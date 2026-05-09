// Sumo robot firmware — Arduino Nano.
// Modules: Config.h (pins/tunables), Sensors, Motors, Strategy.

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Strategy.h"

enum RunMode : uint8_t { MODE_IDLE, MODE_ARMED, MODE_RUNNING, MODE_KILLED };

static RunMode mode = MODE_IDLE;
static volatile bool killed_flag = false;

// ---- Kill ISR ---------------------------------------------------------------
// Drives all motor pins LOW immediately. Avoid PWM/Wire calls in ISR context.
void killISR() {
  digitalWrite(PIN_LEFT_IN1,  LOW);
  digitalWrite(PIN_LEFT_IN2,  LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
  killed_flag = true;
}

// ---- Helpers ---------------------------------------------------------------
static bool buttonPressed() {
  // Active-low button on A1 with INPUT_PULLUP. Debounce.
  if (digitalRead(PIN_START_BTN) != LOW) return false;
  delay(BTN_DEBOUNCE_MS);
  return digitalRead(PIN_START_BTN) == LOW;
}

static bool extStartHigh() {
  return digitalRead(PIN_START_EXT) == HIGH;
}

static void waitForStartTrigger() {
  // Slow heartbeat blink while waiting for arm.
  pinMode(LED_BUILTIN, OUTPUT);
  uint32_t last_toggle = 0;
  bool led = false;
  while (true) {
    if (extStartHigh() || buttonPressed()) return;
    uint32_t now = millis();
    if (now - last_toggle >= 500) {
      led = !led;
      digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
      last_toggle = now;
    }
  }
}

static void countdownThenRun() {
  // Solid LED during countdown.
  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t end = millis() + START_DELAY_MS;
  while ((int32_t)(end - millis()) > 0) {
    if (killed_flag) return;
    delay(10);
  }
}

// ---- Setup -----------------------------------------------------------------
void setup() {
#if SUMO_DEBUG
  Serial.begin(115200);
#endif

  // Inputs.
  pinMode(PIN_START_EXT, INPUT);
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_KILL,      INPUT);   // assumes external pull-down on D3
  pinMode(PIN_LINE_L,    INPUT);
  pinMode(PIN_LINE_R,    INPUT);

  // Motors first so we can guarantee they're off.
  Motors::begin();
  Motors::coast();

  // I2C + ToF bring-up.
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Sensors::initToF();

  // Line sensor thresholds are fixed in Config.h (LINE_L_THRESHOLD /
  // LINE_R_THRESHOLD) — no boot-time calibration.

  // Kill interrupt — attach AFTER motor pins are configured so the ISR's
  // digitalWrites are always valid.
  attachInterrupt(digitalPinToInterrupt(PIN_KILL), killISR, RISING);

#if DEBUG_MOTOR_TEST
  Motors::drive(120, 0);   delay(250);
  Motors::drive(0, 120);   delay(250);
  Motors::drive(-120, -120); delay(250);
  Motors::drive(0, 0);
#endif

  Strategy::reset();

  // Wait for arm signal.
  waitForStartTrigger();
  if (killed_flag) { mode = MODE_KILLED; return; }
  mode = MODE_ARMED;

  // Sumo standard: 5 s grace before first move.
  countdownThenRun();
  if (killed_flag) { mode = MODE_KILLED; return; }
  mode = MODE_RUNNING;
}

// ---- Loop ------------------------------------------------------------------
void loop() {
  if (killed_flag) {
    Motors::coast();
    mode = MODE_KILLED;
#if SUMO_DEBUG
    static bool printed = false;
    if (!printed) { Serial.println(F("KILLED")); printed = true; }
#endif
    return;
  }

  if (mode != MODE_RUNNING) return;

  ToFReadings  tof  = Sensors::readToF();
  LineReadings line = Sensors::readLine();

  SumoState st = Strategy::step(tof, line);
  (void)st;

#if SUMO_DEBUG
  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= 100) {
    last_print = now;
    Serial.print(F("st=")); Serial.print((int)st);
    Serial.print(F(" SL=")); Serial.print(tof.sl);
    Serial.print(F(" FL=")); Serial.print(tof.fl);
    Serial.print(F(" FC=")); Serial.print(tof.fc);
    Serial.print(F(" FR=")); Serial.print(tof.fr);
    Serial.print(F(" SR=")); Serial.print(tof.sr);
    Serial.print(F(" L=")); Serial.print(line.left_raw);
    Serial.print(line.left_white ? F("(W)") : F("(b)"));
    Serial.print(F(" R=")); Serial.print(line.right_raw);
    Serial.println(line.right_white ? F("(W)") : F("(b)"));
  }
#endif
}
