#pragma once
#include <Arduino.h>

namespace Motors {
  // Configure pins. Call once in setup().
  void begin();

  // Sign-magnitude drive. left/right in [-255, +255].
  // Positive = forward, negative = reverse.
  void drive(int left, int right);

  // Hard brake (both pins HIGH on each channel — A4950 brake mode).
  void brake();

  // Coast (all pins LOW). Use when killed.
  void coast();
}
