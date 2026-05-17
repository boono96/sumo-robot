#include "Strategy.h"
#include "Config.h"
#include "Motors.h"

namespace {
  // ---- Persistent state across step() calls ----
  int8_t last_seen_side = 0;   // -1 left, 0 center, +1 right

  // Line escape (latched)
  enum LinePhase : uint8_t { LP_NONE, LP_REVERSE, LP_TURN };
  LinePhase  line_phase = LP_NONE;
  uint32_t   line_phase_end = 0;
  int8_t     line_turn_dir = 0;   // +1 right (CW), -1 left (CCW)

  // Ram stall / collision boost
  bool       pushing = false;
  uint16_t   stall_ref_mm = 0;
  uint32_t   stall_ref_time = 0;
  uint32_t   release_far_since = 0;

  // In-place spin: +1 = clockwise/right, -1 = counter-clockwise/left.
  void spin(int8_t dir, uint8_t pwm) {
    if (dir > 0) Motors::drive(+pwm, -pwm);
    else         Motors::drive(-pwm, +pwm);
  }

  // ---- Behaviors ----

  // Returns true if line maneuver is owning the motors this tick.
  bool handleLineEscape(const LineReadings& line) {
    uint32_t now = millis();

    // If a line maneuver is active, run it to completion.
    if (line_phase == LP_REVERSE) {
      Motors::drive(-RAM_PWM, -RAM_PWM);
      if ((int32_t)(line_phase_end - now) <= 0) {
        // Move to TURN phase.
        line_phase = LP_TURN;
        line_phase_end = now + (line_turn_dir == 0 ? LINE_180_MS : LINE_TURN_MS);
        // line_turn_dir already set when we entered the maneuver.
      }
      return true;
    }
    if (line_phase == LP_TURN) {
      spin(line_turn_dir, TURN_PWM);
      if ((int32_t)(line_phase_end - now) <= 0) {
        line_phase = LP_NONE;
      }
      return true;
    }

    // Not in a maneuver — check for a fresh trip.
    // Convention: line_turn_dir == 0 sentinel means "do a 180 (both-white)";
    //             non-zero means "do a ~120 turn that direction".
    if (line.left_white && line.right_white) {
      line_phase = LP_REVERSE;
      line_phase_end = now + LINE_REVERSE_MS;
      line_turn_dir = 0;
      pushing = false;
      Motors::drive(-RAM_PWM, -RAM_PWM);
      return true;
    }
    if (line.left_white) {
      line_phase = LP_REVERSE;
      line_phase_end = now + LINE_REVERSE_MS;
      line_turn_dir = +1;     // turn right (away from left edge)
      pushing = false;
      Motors::drive(-RAM_PWM, -RAM_PWM);
      return true;
    }
    if (line.right_white) {
      line_phase = LP_REVERSE;
      line_phase_end = now + LINE_REVERSE_MS;
      line_turn_dir = -1;     // turn left (away from right edge)
      pushing = false;
      Motors::drive(-RAM_PWM, -RAM_PWM);
      return true;
    }
    return false;
  }

  // Updates pushing latch using current front-center reading. Returns true if pushing.
  bool updateStallLatch(uint16_t fc) {
    uint32_t now = millis();

    if (!pushing) {
      // Track a moving reference to detect "distance not decreasing while close".
      if (fc < STALL_NEAR_MM) {
        if (stall_ref_time == 0) {
          stall_ref_mm = fc;
          stall_ref_time = now;
        } else if ((int32_t)(now - stall_ref_time) >= STALL_WINDOW_MS) {
          int delta = (int)stall_ref_mm - (int)fc;
          if (delta < (int)STALL_DELTA_MM && delta > -(int)STALL_DELTA_MM) {
            pushing = true;
            release_far_since = 0;
          }
          // restart window
          stall_ref_mm = fc;
          stall_ref_time = now;
        }
      } else {
        stall_ref_time = 0;
      }
      return false;
    }

    // pushing == true: only release on a sustained "far" reading.
    if (fc > STALL_RELEASE_MM) {
      if (release_far_since == 0) release_far_since = now;
      else if ((int32_t)(now - release_far_since) >= STALL_RELEASE_MS) {
        pushing = false;
        stall_ref_time = 0;
        release_far_since = 0;
      }
    } else {
      release_far_since = 0;
    }
    return pushing;
  }
}

void Strategy::reset() {
  last_seen_side = 0;
  line_phase = LP_NONE;
  line_phase_end = 0;
  line_turn_dir = 0;
  pushing = false;
  stall_ref_mm = 0;
  stall_ref_time = 0;
  release_far_since = 0;
}

SumoState Strategy::step(const ToFReadings& tof, const LineReadings& line) {
  // 1. Line escape preempts everything.
  if (handleLineEscape(line)) return STATE_LINE_ESCAPE;

  // 2. Ram (with stall latch).
  bool latched = updateStallLatch(tof.fc);
  if (latched || tof.fc < RAM_DISTANCE) {
    Motors::drive(+RAM_PWM, +RAM_PWM);
    last_seen_side = 0;
    return STATE_RAM;
  }

  // 3. Track — FC is the alignment indicator. Commit forward when FC has any
  //    lock; only curve hard when FC sees nothing. FC_BIAS_MM keeps us
  //    committed to a straight ram on small left/right asymmetries.
  if (tof.fc < ENGAGE_DISTANCE) {
    // FC has a target — drive straight unless FL or FR reads MUCH closer
    // (real angular offset), in which case nudge the bearing slightly.
    bool fl_lead = (uint32_t)tof.fl + FC_BIAS_MM < (uint32_t)tof.fc;
    bool fr_lead = (uint32_t)tof.fr + FC_BIAS_MM < (uint32_t)tof.fc;
    if (fl_lead && !fr_lead) {
      Motors::drive(+CURVE_SLOW_PWM, +CURVE_FAST_PWM);
      last_seen_side = -1;
    } else if (fr_lead && !fl_lead) {
      Motors::drive(+CURVE_FAST_PWM, +CURVE_SLOW_PWM);
      last_seen_side = +1;
    } else {
      Motors::drive(+FORWARD_PWM, +FORWARD_PWM);
      last_seen_side = 0;
    }
    return STATE_TRACK;
  }
  if (tof.fl < ENGAGE_DISTANCE || tof.fr < ENGAGE_DISTANCE) {
    // FC blind, opponent genuinely off-axis — turn toward whichever side reads.
    if (tof.fl <= tof.fr) {
      Motors::drive(+CURVE_SLOW_PWM, +CURVE_FAST_PWM);
      last_seen_side = -1;
    } else {
      Motors::drive(+CURVE_FAST_PWM, +CURVE_SLOW_PWM);
      last_seen_side = +1;
    }
    return STATE_TRACK;
  }

  // 4. Side sensor seeing something inside ENGAGE — bias the next search
  //    direction so we spin toward whichever side last saw the opponent.
  if (tof.sl < ENGAGE_DISTANCE && tof.sl <= tof.sr) {
    last_seen_side = -1;
  } else if (tof.sr < ENGAGE_DISTANCE) {
    last_seen_side = +1;
  }

  // 5. Search — spin toward last seen side (default left if center/unknown).
  int8_t dir = (last_seen_side > 0) ? +1 : -1;
  spin(dir, SEARCH_PWM);
  return STATE_SEARCH;
}
