#include "flight_sm.h"
#include "guidance.h"
#include "nav_math.h"
#include <math.h>

FlightStateMachine::FlightStateMachine() {}

__attribute__((noinline))
void FlightStateMachine::reset() {
    current_state = FlightState::BOOT;
    max_altitude = 0.0f;
    last_altitude = 0.0f;
    state_entry_time_ms = 0;
    still_start_time_ms = 0;
    burnwire_start_ms = 0;
    burnwire_active = false;
    buzzer_active = false;
    last_breakout_alt = 0.0f;
}

void FlightStateMachine::transitionTo(FlightState next_state, uint32_t now_ms) {
    if (current_state == next_state) return;
    current_state = next_state;
    state_entry_time_ms = now_ms;
    still_start_time_ms = 0;

    if (next_state == FlightState::PARAFOIL_DEPLOYMENT) {
        burnwire_active = true;
        burnwire_start_ms = now_ms;
        buzzer_active = false;
    } else {
        burnwire_active = false;
        buzzer_active = (next_state == FlightState::LANDING || next_state == FlightState::FAILSAFE);
    }
}

void FlightStateMachine::forceState(FlightState next_state, uint32_t now_ms) {
    current_state = FlightState::BOOT;
    transitionTo(next_state, now_ms);
}

void FlightStateMachine::tickTimers(uint32_t now_ms) {
    if (burnwire_active && (now_ms - burnwire_start_ms >= BURNWIRE_DURATION_MS)) {
        burnwire_active = false;
    }
    // Auto-advance out of deployment into guidance once 30s pulse has elapsed
    if (current_state == FlightState::PARAFOIL_DEPLOYMENT) {
        if (now_ms - state_entry_time_ms >= BURNWIRE_DURATION_MS) {
            transitionTo(FlightState::PRECISION_GUIDANCE, now_ms);
        }
    }
}

bool FlightStateMachine::isSteeringEnabled() const {
    return (current_state == FlightState::PRECISION_GUIDANCE ||
            current_state == FlightState::TERMINAL_GUIDANCE ||
            current_state == FlightState::FINAL_APPROACH);
}

bool FlightStateMachine::isFlywheelActive() const {
    // Runs at constant 6000 RPM in DROGUE_DESCENT, PARAFOIL_DEPLOYMENT, and GUIDANCE until LANDING
    return (current_state == FlightState::DROGUE_DESCENT ||
            current_state == FlightState::PARAFOIL_DEPLOYMENT ||
            current_state == FlightState::PRECISION_GUIDANCE ||
            current_state == FlightState::TERMINAL_GUIDANCE ||
            current_state == FlightState::FINAL_APPROACH);
}

__attribute__((noinline))
void FlightStateMachine::update(
    float alt_agl,
    float heading,
    const LocalPoint& pos,
    const LocalPoint& target,
    float /*dt*/,
    uint32_t now_ms
) {
    if (state_entry_time_ms == 0) state_entry_time_ms = now_ms;

    // Track peak altitude
    if (alt_agl > max_altitude) {
        max_altitude = alt_agl;
    }

    // Burnwire 30s pulse cutoff
    if (burnwire_active && (now_ms - burnwire_start_ms >= BURNWIRE_DURATION_MS)) {
        burnwire_active = false;
    }

    float target_dist = distance_xy(pos, target);

    switch (current_state) {
        case FlightState::BOOT:
            // Move immediately to LAUNCH_PAD once booted
            transitionTo(FlightState::LAUNCH_PAD, now_ms);
            break;

        case FlightState::LAUNCH_PAD:
            // Detect launch climb
            if (alt_agl > LAUNCH_DETECT_ALT) {
                transitionTo(FlightState::ASCENT, now_ms);
            }
            break;

        case FlightState::ASCENT:
            // Drone drop detection:
            // The drone ascends smoothly. When dropped:
            // 1) Peak altitude is at least 30m (so ground handling doesn't trigger it)
            // 2) CanSat has dropped at least 10m below peak altitude
            // 3) If release is at or below PARAFOIL_DEPLOY_ALT (700m), immediately trigger deployment!
            //    If release is above 700m (e.g. 800m-1000m), enter DROGUE_DESCENT until 700m.
            if (max_altitude >= 30.0f && alt_agl < (max_altitude - 10.0f)) {
                if (alt_agl <= PARAFOIL_DEPLOY_ALT) {
                    transitionTo(FlightState::PARAFOIL_DEPLOYMENT, now_ms);
                } else {
                    transitionTo(FlightState::DROGUE_DESCENT, now_ms);
                }
            }
            break;

        case FlightState::DROGUE_DESCENT:
            // Reefed parachute descent: trigger disreefing when reaching 700m AGL
            if (alt_agl <= PARAFOIL_DEPLOY_ALT) {
                transitionTo(FlightState::PARAFOIL_DEPLOYMENT, now_ms);
            }
            break;

        case FlightState::PARAFOIL_DEPLOYMENT:
            // Pulse burnwire for 30s before heading to precision guidance
            if ((now_ms - state_entry_time_ms >= BURNWIRE_DURATION_MS) || alt_agl <= TOUCHDOWN_ALT) {
                burnwire_active = false;
                transitionTo(FlightState::PRECISION_GUIDANCE, now_ms);
            }
            break;

        case FlightState::PRECISION_GUIDANCE:
            // Immediate touchdown safety catch
            if (alt_agl <= TOUCHDOWN_ALT) {
                transitionTo(FlightState::LANDING, now_ms);
                buzzer_active = true;
                break;
            }
            // Direct final approach condition
            if (alt_agl <= FINAL_APPROACH_ALT && target_dist <= FINAL_APPROACH_RADIUS) {
                transitionTo(FlightState::FINAL_APPROACH, now_ms);
            }
            // Energy-excess terminal holding orbit condition
            else if (shouldEnterTerminalGuidance(alt_agl, target_dist, 0.0f, 0.0f, bearing(pos, target))) {
                transitionTo(FlightState::TERMINAL_GUIDANCE, now_ms);
            }
            break;

        case FlightState::TERMINAL_GUIDANCE: {
            if (alt_agl <= TOUCHDOWN_ALT) {
                transitionTo(FlightState::LANDING, now_ms);
                buzzer_active = true;
                break;
            }
            BreakoutGeometry geom = calculateBreakoutGeometry(pos, heading, target, 0.0f, 0.0f);
            last_breakout_alt = geom.breakout_altitude;

            // Watchdog: max 120s in holding orbit
            if (now_ms - state_entry_time_ms > 120000) {
                transitionTo(FlightState::FINAL_APPROACH, now_ms);
            }
            // Dynamic energy breakout trigger
            else if (alt_agl <= geom.breakout_altitude) {
                transitionTo(FlightState::FINAL_APPROACH, now_ms);
            }
            break;
        }

        case FlightState::FINAL_APPROACH:
            // Touchdown detection: altitude below 2m or ground contact
            if (alt_agl <= TOUCHDOWN_ALT) {
                transitionTo(FlightState::LANDING, now_ms);
                buzzer_active = true;
            }
            break;

        case FlightState::LANDING:
            // Mission complete, buzzer active, continuous beaconing
            buzzer_active = true;
            break;

        case FlightState::FAILSAFE:
            // Emergency safe state
            buzzer_active = true;
            break;
    }

    // Global Ground Stillness Detection (backup landing trigger in final approach)
    if (current_state == FlightState::FINAL_APPROACH) {
        if (fabsf(alt_agl - last_altitude) < 0.05f && alt_agl < 5.0f) {
            if (still_start_time_ms == 0) still_start_time_ms = now_ms;
            if (now_ms - still_start_time_ms >= TOUCHDOWN_STILL_TIME_MS) {
                transitionTo(FlightState::LANDING, now_ms);
                buzzer_active = true;
            }
        } else {
            still_start_time_ms = 0;
        }
    }

    last_altitude = alt_agl;
}
