#pragma once

// MADS (Modular Assistive Driving System) lateral control state machine,
// ported from sunnypilot (opendbc/safety/sunnypilot/mads.h) and translated to
// this fork's SafetyState/SafetySetup architecture.
//
// The machine decides safety.lateral_controls_allowed from edges of the ACC
// main state, openpilot's controls_allowed, brake presses and steering
// disengages, so lateral control can stay engaged independently of
// longitudinal control. It is gated on the ENABLE_MADS alternative
// experience bit, which openpilot sets from the user's MADS settings.
//
// Deviations from sunnypilot:
// - Fully inert unless ENABLE_MADS is set, so safety modes that drive
//   lateral_controls_allowed themselves (e.g. Toyota) are never affected.
//   Sunny runs the machine unconditionally and only gates the final write.
// - No panda-side MADS button tracking: in this architecture, brand button
//   presses surface as openpilot button events, not as CAN state tracked by
//   the panda. Lateral requests rise on ACC main / controls_allowed edges.
// - The system/brake modes are read from alternative_experience on every
//   cycle instead of being latched when the experience is set.

#define ALT_EXP_ENABLE_MADS 1024
#define ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE 2048
#define ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE 4096

static bool mads_system_enabled(void) {
  return (safety_setup.alternative_experience & ALT_EXP_ENABLE_MADS) != 0;
}

static MadsEdgeTransition mads_get_edge_transition(bool current, bool last) {
  MadsEdgeTransition state = MADS_EDGE_NO_CHANGE;
  if (current && !last) {
    state = MADS_EDGE_RISING;
  } else if (!current && last) {
    state = MADS_EDGE_FALLING;
  }
  return state;
}

static void mads_update_binary_state(MadsBinaryStateTracking *state) {
  state->transition = mads_get_edge_transition(state->current, state->previous);
  state->previous = state->current;
}

static void mads_exit_controls(uint32_t reason) {
  // Always track this as a pending reason
  safety.mads.pending_reasons |= reason;

  if (safety.lateral_controls_allowed) {
    safety.mads.active_reason = reason;
    safety.mads.controls_requested_lateral = false;
    safety.lateral_controls_allowed = false;
  }
}

static void mads_update_control_state(void) {
  const bool disengage_on_brake = (safety_setup.alternative_experience & ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE) != 0;
  const bool pause_on_brake = (safety_setup.alternative_experience & ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE) != 0;
  bool allowed = true;

  // Initial control requests from ACC main or openpilot engagement transitions
  if ((safety.mads.acc_main.transition == MADS_EDGE_RISING) ||
      (safety.mads.op_controls_allowed.transition == MADS_EDGE_RISING)) {
    safety.mads.controls_requested_lateral = true;
  }

  // Primary control blockers - these prevent any further control processing
  if (safety.mads.acc_main.transition == MADS_EDGE_FALLING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (safety.mads.steering_disengage.transition == MADS_EDGE_RISING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (disengage_on_brake && (safety.mads.braking.transition == MADS_EDGE_RISING)) {
    mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
    allowed = false;
  }

  // Secondary control conditions - only checked if primary conditions don't block further control processing
  if (allowed && pause_on_brake) {
    // Brake rising edge immediately blocks controls
    // Brake release might request controls if brake was the ONLY reason for disengagement
    if (safety.mads.braking.transition == MADS_EDGE_RISING) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
      allowed = false;
    } else if ((safety.mads.braking.transition == MADS_EDGE_FALLING) &&
               (safety.mads.active_reason == (uint32_t)MADS_DISENGAGE_REASON_BRAKE) &&
               (safety.mads.pending_reasons == (uint32_t)MADS_DISENGAGE_REASON_BRAKE)) {
      safety.mads.controls_requested_lateral = true;
    } else if (safety.mads.braking.current) {
      allowed = false;
    }
  }

  // Process control request if conditions allow
  if (allowed && safety.mads.controls_requested_lateral && !safety.lateral_controls_allowed) {
    safety.mads.controls_requested_lateral = false;
    safety.lateral_controls_allowed = true;
    safety.mads.active_reason = MADS_DISENGAGE_REASON_NONE;
    safety.mads.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  }
}

static void mads_state_update(void) {
  if (!mads_system_enabled()) {
    // Runtime disable after the machine engaged lateral: hand control back cleanly.
    // Modes that never enabled MADS keep full ownership of lateral_controls_allowed.
    if (safety.mads.active) {
      safety.mads = (MadsState){0};
      safety.lateral_controls_allowed = false;
    }
    return;
  }
  safety.mads.active = true;

  safety.mads.acc_main.current = safety.acc_main_on;
  safety.mads.op_controls_allowed.current = safety.controls_allowed;
  safety.mads.braking.current = safety.brake_pressed || safety.regen_braking;
  safety.mads.steering_disengage.current = safety.steering_disengage;

  mads_update_binary_state(&safety.mads.acc_main);
  mads_update_binary_state(&safety.mads.op_controls_allowed);
  mads_update_binary_state(&safety.mads.braking);
  mads_update_binary_state(&safety.mads.steering_disengage);

  mads_update_control_state();
}

static void mads_heartbeat_engaged_check(void) {
  if (!mads_system_enabled()) {
    safety.heartbeat_engaged_mads_mismatches = 0U;
    return;
  }

  // exit lateral controls allowed if MADS is unused by openpilot for a few cycles
  if (safety.lateral_controls_allowed && !safety.heartbeat_engaged_mads) {
    safety.heartbeat_engaged_mads_mismatches += 1U;
    if (safety.heartbeat_engaged_mads_mismatches >= 3U) {
      mads_exit_controls(MADS_DISENGAGE_REASON_HEARTBEAT_ENGAGED_MISMATCH);
    }
  } else {
    safety.heartbeat_engaged_mads_mismatches = 0U;
  }
}
