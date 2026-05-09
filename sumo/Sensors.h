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

  // Sample both line sensors over the (assumed) black surface
  // for LINE_CAL_MS to learn a baseline. Threshold = baseline + LINE_MARGIN.
  void calibrateLineBaseline();

  // Read all five distances (mm). Out-of-range / no fresh data clamped to TOF_NO_TARGET.
  ToFReadings readToF();

  // Read line sensors. left_white/right_white set per calibrated threshold.
  LineReadings readLine();
}
