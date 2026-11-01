#include "flight_sm.h"
#include <math.h>

FlightStateMachine::FlightStateMachine() {}

__attribute__((noinline))
void FlightStateMachine::reset() {
    current_state = FlightState::BOOT;
    max_altitude = 0.0f;
    state_entry_time_ms = 0;
    still_start_time_ms = 0;
    burnwire_start_ms = 0;
    slow_sink_start_ms = 0;
    inflated_ms = 0;
    drogue_rate = DROGUE_DESCENT_RATE;
    canopy_open_alt = 0.0f;
    burnwire_on_alt = 0.0f;
    burnwire_active = false;
    buzzer_active = false;
    canopy_inflated = false;
    seen_fast_fall = false;
}

float FlightStateMachine::getDeployTriggerAlt() const {
    // Switch the burnwire on early by (drogue speed x cut time) so the line parts at 600 m.
    return PARAFOIL_DEPLOY_ALT;
}

void FlightStateMachine::transitionTo(FlightState next_state, uint32_t now_ms, float alt_agl) {
    if (current_state == next_state) return;
    current_state = next_state;
    state_entry_time_ms = now_ms;
    still_start_time_ms = 0;

    if (next_state == FlightState::PARAFOIL_DEPLOYMENT) {
        burnwire_active = true;
        burnwire_start_ms = now_ms;
        burnwire_on_alt = alt_agl;
        slow_sink_start_ms = 0;
    } else if (next_state < FlightState::PARAFOIL_DEPLOYMENT || next_state >= FlightState::LANDING) {
        burnwire_active = false;
    }
    buzzer_active = (next_state == FlightState::LANDING || next_state == FlightState::FAILSAFE);
}

void FlightStateMachine::forceState(FlightState next_state, uint32_t now_ms) {
    FlightState prev = current_state;
    current_state = FlightState::BOOT;
    if (next_state <= FlightState::PARAFOIL_DEPLOYMENT) {
        canopy_inflated = false;
        seen_fast_fall = false;
    } else if (next_state < FlightState::LANDING) {
        canopy_inflated = true;      // commanded straight into guidance: canopy assumed open
    }
    transitionTo(next_state, now_ms, 0.0f);
    if (prev == next_state) state_entry_time_ms = now_ms;
}

void FlightStateMachine::tickTimers(uint32_t now_ms) {
    if (burnwire_active && (now_ms - burnwire_start_ms >= BURNWIRE_DURATION_MS)) {
        burnwire_active = false;
    }
    if (current_state == FlightState::PARAFOIL_DEPLOYMENT &&
        (now_ms - state_entry_time_ms >= DEPLOY_TIMEOUT_MS)) {
        canopy_inflated = true;      // timeout: assume open so the flywheel still starts
        burnwire_active = false;
        transitionTo(FlightState::PRECISION_GUIDANCE, now_ms, 0.0f);
    }
}

bool FlightStateMachine::isSteeringEnabled() const {
    return (current_state == FlightState::PRECISION_GUIDANCE ||
            current_state == FlightState::TERMINAL_GUIDANCE ||
            current_state == FlightState::FINAL_APPROACH);
}

bool FlightStateMachine::isFlywheelActive() const {
    // Spin up only once the canopy is open: the two-riser suspension then
    // resists the spin-up reaction torque (a few degrees of twist).  Spinning
    // up under the single-point drogue would counter-rotate the can (~100+ rpm)
    // and wind twists into the packed parafoil lines.
    return canopy_inflated &&
           current_state >= FlightState::PARAFOIL_DEPLOYMENT &&
           current_state <= FlightState::FINAL_APPROACH;
}

void FlightStateMachine::requestGuidanceState(FlightState s, uint32_t now_ms) {
    if (!isSteeringEnabled()) return;
    if (s != FlightState::PRECISION_GUIDANCE &&
        s != FlightState::TERMINAL_GUIDANCE &&
        s != FlightState::FINAL_APPROACH) return;
    transitionTo(s, now_ms, 0.0f);
}

__attribute__((noinline))
void FlightStateMachine::update(float alt_agl, float climb_rate, uint32_t now_ms) {
    if (state_entry_time_ms == 0) state_entry_time_ms = now_ms;
    if (alt_agl > max_altitude) max_altitude = alt_agl;

    float sink = -climb_rate;
    if (sink > FAST_FALL_RATE) seen_fast_fall = true;

    if (burnwire_active && (now_ms - burnwire_start_ms >= BURNWIRE_DURATION_MS)) {
        burnwire_active = false;
    }

    switch (current_state) {
        case FlightState::BOOT:
            transitionTo(FlightState::LAUNCH_PAD, now_ms, alt_agl);
            break;

        case FlightState::LAUNCH_PAD:
            if (alt_agl > LAUNCH_DETECT_ALT) {
                transitionTo(FlightState::ASCENT, now_ms, alt_agl);
            }
            break;

        case FlightState::ASCENT:
            // Release from the drone: well below the peak AND falling fast.
            if (max_altitude >= RELEASE_MIN_PEAK_ALT &&
                alt_agl < (max_altitude - RELEASE_DROP_ALT) &&
                sink > RELEASE_FALL_RATE) {
                transitionTo((alt_agl <= getDeployTriggerAlt()) ? FlightState::PARAFOIL_DEPLOYMENT
                                                                : FlightState::DROGUE_DESCENT,
                             now_ms, alt_agl);
            }
            break;

        case FlightState::DROGUE_DESCENT:
            // Measure the real drogue speed (used to time the burnwire).
            if (sink > 8.0f && sink < 30.0f && (now_ms - state_entry_time_ms) > 3000UL) {
                drogue_rate += 0.05f * (sink - drogue_rate);
            }
            if (alt_agl <= getDeployTriggerAlt()) {
                transitionTo(FlightState::PARAFOIL_DEPLOYMENT, now_ms, alt_agl);
            }
            break;

        case FlightState::PARAFOIL_DEPLOYMENT:
            // Canopy open = sink rate drops by >= BURNWIRE_VEL_DROP_THRESH or below INFLATION_SINK_RATE.
            if (seen_fast_fall && !canopy_inflated) {
                float vel_drop = drogue_rate - sink;
                if (vel_drop >= BURNWIRE_VEL_DROP_THRESH || sink < INFLATION_SINK_RATE) {
                    if (slow_sink_start_ms == 0) slow_sink_start_ms = now_ms;
                    if (now_ms - slow_sink_start_ms >= INFLATION_CONFIRM_MS) {
                        canopy_inflated = true;
                        inflated_ms = now_ms;
                        canopy_open_alt = alt_agl;
                        burnwire_active = false;     // stop heating next to the PE lines
                    }
                } else {
                    slow_sink_start_ms = 0;
                }
            }
            // Start up servos and enter precision guidance when descending below SERVO_START_ALT (500m AGL)
            if (((canopy_inflated && now_ms - inflated_ms >= CANOPY_SETTLE_MS) || (now_ms - state_entry_time_ms >= DEPLOY_TIMEOUT_MS)) && (alt_agl <= SERVO_START_ALT)) {
                canopy_inflated = true;
                burnwire_active = false;
                transitionTo(FlightState::PRECISION_GUIDANCE, now_ms, alt_agl);
            } else if (alt_agl <= TOUCHDOWN_ALT && seen_fast_fall) {
                transitionTo(FlightState::LANDING, now_ms, alt_agl);
            }
            break;

        case FlightState::PRECISION_GUIDANCE:
        case FlightState::TERMINAL_GUIDANCE:
        case FlightState::FINAL_APPROACH:
            if (alt_agl <= TOUCHDOWN_ALT) {
                transitionTo(FlightState::LANDING, now_ms, alt_agl);
                break;
            }
            // Backup touchdown: stationary near the ground (covers baro drift).
            if (alt_agl < TOUCHDOWN_STILL_ALT && fabsf(climb_rate) < TOUCHDOWN_STILL_RATE) {
                if (still_start_time_ms == 0) still_start_time_ms = now_ms;
                if (now_ms - still_start_time_ms >= TOUCHDOWN_STILL_TIME_MS) {
                    transitionTo(FlightState::LANDING, now_ms, alt_agl);
                }
            } else {
                still_start_time_ms = 0;
            }
            break;

        case FlightState::LANDING:
        case FlightState::FAILSAFE:
            buzzer_active = true;
            break;
    }
}
