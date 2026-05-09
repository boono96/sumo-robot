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

  // Anti-flank (latched once committed)
  uint32_t   flank_end = 0;
  int8_t     flank_dir = 0;       // +1 right, -1 left
  // Pre-commit persistence: when did each side first satisfy the flank predicate?
  uint32_t   flank_l_since = 0;
  uint32_t   flank_r_since = 0;

  // Ram stall / collision boost
  bool       pushing = false;
  uint16_t   stall_ref_mm = 0;
  uint32_t   stall_ref_time = 0;
  uint32_t   release_far_since = 0;

  inline uint16_t minOf3(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t m = a < b ? a : b;
    return m < c ? m : c;
  }

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
      flank_end = 0;
      Motors::drive(-RAM_PWM, -RAM_PWM);
      return true;
    }
    if (line.left_white) {
      line_phase = LP_REVERSE;
      line_phase_end = now + LINE_REVERSE_MS;
      line_turn_dir = +1;     // turn right (away from left edge)
      pushing = false;
      flank_end = 0;
      Motors::drive(-RAM_PWM, -RAM_PWM);
      return true;
    }
    if (line.right_white) {
      line_phase = LP_REVERSE;
      line_phase_end = now + LINE_REVERSE_MS;
      line_turn_dir = -1;     // turn left (away from right edge)
      pushing = false;
      flank_end = 0;
      Motors::drive(-RAM_PWM, -RAM_PWM);
      return true;
    }
    return false;
  }

  // Predicate: side reading represents a real flank threat (not banner sweep).
  // Requires the side to be close AND meaningfully closer than every front
  // reading. A banner whose body sits ahead will leave at least one front
  // sensor seeing the body at similar-or-closer range than the side.
  inline bool isRealFlank(uint16_t side, const ToFReadings& tof) {
    if (side >= FLANK_CLOSE_MM) return false;
    uint16_t front_min = minOf3(tof.fl, tof.fc, tof.fr);
    return (uint32_t)side + FLANK_LEAD_MM < (uint32_t)front_min;
  }

  // Returns true if anti-flank maneuver is owning the motors this tick.
  bool handleAntiFlank(const ToFReadings& tof) {
    uint32_t now = millis();

    // Active latch — keep spinning until front center sees target or timeout.
    if (flank_end != 0 && (int32_t)(flank_end - now) > 0) {
      if (tof.fc < ENGAGE_DISTANCE) {
        flank_end = 0;
        return false;
      }
      spin(flank_dir, TURN_PWM);
      return true;
    }
    flank_end = 0;

    // Update per-side "since" timestamps for persistence gating.
    bool l_real = isRealFlank(tof.sl, tof);
    bool r_real = isRealFlank(tof.sr, tof);
    if (l_real) { if (flank_l_since == 0) flank_l_since = now; }
    else        { flank_l_since = 0; }
    if (r_real) { if (flank_r_since == 0) flank_r_since = now; }
    else        { flank_r_since = 0; }

    // Trigger only after the predicate has held for FLANK_PERSIST_MS — this
    // filters out a swinging banner that briefly enters the side beam.
    bool l_trig = flank_l_since != 0 && (now - flank_l_since) >= FLANK_PERSIST_MS;
    bool r_trig = flank_r_since != 0 && (now - flank_r_since) >= FLANK_PERSIST_MS;

    if (l_trig && (!r_trig || tof.sl <= tof.sr)) {
      flank_dir = -1;
      flank_end = now + ANTI_FLANK_MS;
      pushing = false;
      flank_l_since = 0;
      spin(flank_dir, TURN_PWM);
      return true;
    }
    if (r_trig) {
      flank_dir = +1;
      flank_end = now + ANTI_FLANK_MS;
      pushing = false;
      flank_r_since = 0;
      spin(flank_dir, TURN_PWM);
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
  flank_end = 0;
  flank_dir = 0;
  flank_l_since = 0;
  flank_r_since = 0;
  pushing = false;
  stall_ref_mm = 0;
  stall_ref_time = 0;
  release_far_since = 0;
}

SumoState Strategy::step(const ToFReadings& tof, const LineReadings& line) {
  // 1. Line escape preempts everything.
  if (handleLineEscape(line)) return STATE_LINE_ESCAPE;

  // 2. Anti-flank preempts attack.
  if (handleAntiFlank(tof)) return STATE_ANTI_FLANK;

  // 3. Ram (with stall latch).
  bool latched = updateStallLatch(tof.fc);
  if (latched || tof.fc < RAM_DISTANCE) {
    Motors::drive(+RAM_PWM, +RAM_PWM);
    last_seen_side = 0;
    return STATE_RAM;
  }

  // 4. Track — FC is the body indicator. Commit forward when FC has any lock,
  //    only curve hard when FC sees nothing. FL/FR alone are treated as
  //    suspect (could be banner sweep).
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

  // 5. Side sensor seeing something inside ENGAGE — only used to bias the
  //    next search direction, never to act. Banner readings end up here too,
  //    but a one-frame bias just sets which way we'll spin in SEARCH and is
  //    harmless if wrong.
  if (tof.sl < ENGAGE_DISTANCE && tof.sl <= tof.sr) {
    last_seen_side = -1;
  } else if (tof.sr < ENGAGE_DISTANCE) {
    last_seen_side = +1;
  }

  // 6. Search — spin toward last seen side (default left if center/unknown).
  int8_t dir = (last_seen_side > 0) ? +1 : -1;
  spin(dir, SEARCH_PWM);
  return STATE_SEARCH;
}
