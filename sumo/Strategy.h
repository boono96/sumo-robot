#pragma once
#include <Arduino.h>
#include "Sensors.h"

enum SumoState : uint8_t {
  STATE_LINE_ESCAPE,
  STATE_ANTI_FLANK,
  STATE_RAM,
  STATE_TRACK,
  STATE_SEARCH,
};

namespace Strategy {
  void reset();
  // Returns the chosen state for telemetry/debug. Drives motors as a side effect.
  SumoState step(const ToFReadings& tof, const LineReadings& line);
}
