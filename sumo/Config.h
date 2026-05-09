#pragma once
#include <Arduino.h>

// ---- ToF XSHUT pins ----
constexpr uint8_t PIN_XSHUT_SL = 8;
constexpr uint8_t PIN_XSHUT_FL = 12;
constexpr uint8_t PIN_XSHUT_FC = 4;
constexpr uint8_t PIN_XSHUT_FR = 2;
constexpr uint8_t PIN_XSHUT_SR = 7;

// ---- ToF re-assigned I2C addresses ----
constexpr uint8_t TOF_ADDR_SL = 0x30;
constexpr uint8_t TOF_ADDR_FL = 0x31;
constexpr uint8_t TOF_ADDR_FC = 0x32;
constexpr uint8_t TOF_ADDR_FR = 0x33;
constexpr uint8_t TOF_ADDR_SR = 0x34;

// ---- Line sensors (analog) ----
constexpr uint8_t PIN_LINE_R = A2;
constexpr uint8_t PIN_LINE_L = A7;

// ---- Start / kill control ----
constexpr uint8_t PIN_START_EXT = A0;     // digital read, HIGH = start
constexpr uint8_t PIN_START_BTN = A1;     // active-low button (INPUT_PULLUP)
constexpr uint8_t PIN_KILL      = 3;      // INT1 on Nano

// ---- Motor driver (A4950) ----
// Channel A = RIGHT motor, Channel B = LEFT motor
// IN1 = motor +, IN2 = motor -
constexpr uint8_t PIN_RIGHT_IN1 = 6;
constexpr uint8_t PIN_RIGHT_IN2 = 5;
constexpr uint8_t PIN_LEFT_IN1  = 10;
constexpr uint8_t PIN_LEFT_IN2  = 11;

// ---- Distances (mm) ----
constexpr uint16_t ENGAGE_DISTANCE = 700;
constexpr uint16_t RAM_DISTANCE    = 200;
constexpr uint16_t TOF_NO_TARGET   = 800;   // anything beyond = treat as nothing seen

// ---- Banner discrimination ----
// Some opponents have a wide swinging banner on a stick. Its readings appear on
// our side beams (and sometimes FL/FR) at moderate range while the body sits
// straight ahead. To avoid being yanked off the body, side-flank actions
// require the side reading to be both close AND meaningfully closer than every
// front reading, AND to persist for FLANK_PERSIST_MS.
constexpr uint16_t FLANK_CLOSE_MM    = 200;   // side must be at least this close
constexpr uint16_t FLANK_LEAD_MM     = 150;   // and this much closer than min(FL,FC,FR)
constexpr uint16_t FLANK_PERSIST_MS  = 60;

// In TRACK, FC is the "body" indicator. FL/FR are only allowed to override FC's
// straight-ahead command when they read this much closer than FC — otherwise
// FL/FR readings are treated as banner noise and we keep driving straight.
constexpr uint16_t FC_BIAS_MM        = 150;

// ---- Line sensor thresholds (pre-calibrated, no boot-time sampling) ----
// analogRead counts; reading > threshold => WHITE. Tune per-sensor on the
// real arena surface and edit here. Robot does NOT calibrate at startup.
constexpr int      LINE_L_THRESHOLD = 600;
constexpr int      LINE_R_THRESHOLD = 600;

// ---- PWM (0-255) ----
constexpr uint8_t  RAM_PWM         = 255;
constexpr uint8_t  FORWARD_PWM     = 200;
constexpr uint8_t  TURN_PWM        = 200;
constexpr uint8_t  SEARCH_PWM      = 160;
constexpr uint8_t  CURVE_FAST_PWM  = 220;
constexpr uint8_t  CURVE_SLOW_PWM  = 120;

// ---- Stall / collision boost ----
constexpr uint16_t STALL_NEAR_MM    = 150;
constexpr uint16_t STALL_DELTA_MM   = 15;
constexpr uint16_t STALL_WINDOW_MS  = 120;
constexpr uint16_t STALL_RELEASE_MM = 350;
constexpr uint16_t STALL_RELEASE_MS = 80;

// ---- Timings (ms) ----
constexpr uint16_t LINE_REVERSE_MS = 180;
constexpr uint16_t LINE_TURN_MS    = 250;   // ~120 deg in-place turn
constexpr uint16_t LINE_180_MS     = 380;
constexpr uint16_t START_DELAY_MS  = 10;  // sumo standard 5 s
constexpr uint16_t BTN_DEBOUNCE_MS = 30;
constexpr uint16_t ANTI_FLANK_MS   = 300;

// ---- Sensor timing ----
constexpr uint32_t TOF_TIMING_BUDGET_US = 20000;   // 20 ms = high-speed preset
constexpr uint32_t I2C_CLOCK_HZ         = 400000;

// ---- Debug ----
// Set to 1 to print state + distances to Serial @ 115200.
#define SUMO_DEBUG 0
// Set to 1 for the brief motor twitch test in setup().
#define DEBUG_MOTOR_TEST 0
// Set to 1 to trace VL53L0X bring-up. On Serial, prints which sensor is being
// brought up at each step (last line printed = the one that hung). On the
// onboard LED, a stuck sensor flashes N short pulses then a long pause where
// N = 1..5 for SL, FL, FC, FR, SR — so you can identify the failing sensor
// even without a serial monitor connected.
#define VL53L0X_DEBUG 1
