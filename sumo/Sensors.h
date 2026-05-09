#pragma once
#include <Arduino.h>

struct ToFReadings {
  uint16_t sl;   // side left
  uint16_t fl;   // front left
  uint16_t fc;   // front center
  uint16_t fr;   // front right
  uint16_t sr;   // side right
};

struct LineReadings {
  bool left_white;
  bool right_white;
  int  left_raw;
  int  right_raw;
};

namespace Sensors {
  // Bring up all five VL53L0X sensors with re-assigned addresses.
  // Call once in setup() AFTER Wire.begin().
  void initToF();

  // Read all five distances (mm). Out-of-range / no fresh data clamped to TOF_NO_TARGET.
  ToFReadings readToF();

  // Read line sensors. left_white/right_white compared against the fixed
  // LINE_L_THRESHOLD / LINE_R_THRESHOLD constants in Config.h.
  LineReadings readLine();
}
