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
    filtered_descent_rate = 0.0f;
    deploy_trigger_vz = 0.0f;
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
        deploy_trigger_vz = (filtered_descent_rate > 5.0f) ? filtered_descent_rate : DROGUE_DESCENT_RATE;
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
    if (burnwire_active && (now_ms - burnwire_start_ms >= BURNWIRE_MAX_DURATION_MS)) {
        burnwire_active = false;
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
    float dt,
    uint32_t now_ms
) {
    if (state_entry_time_ms == 0) state_entry_time_ms = now_ms;

    // Track peak altitude
    if (alt_agl > max_altitude) {
        max_altitude = alt_agl;
    }

    // Update estimated descent rate (m/s)
    if (last_altitude > 0.0f && dt > 0.001f) {
        float raw_vz = (last_altitude - alt_agl) / dt;
        if (raw_vz < -20.0f) raw_vz = -20.0f;
        if (raw_vz > 50.0f) raw_vz = 50.0f;
        filtered_descent_rate = (filtered_descent_rate == 0.0f) ? raw_vz : (0.25f * raw_vz + 0.75f * filtered_descent_rate);
    }

    // Safety thermal cutoff for burnwire
    if (burnwire_active && (now_ms - burnwire_start_ms >= BURNWIRE_MAX_DURATION_MS)) {
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
            // Drone drop / apogee release detection:
            // When release occurs at or below 750m, immediately trigger deployment.
            // When release occurs above 750m (e.g. 800m-1000m), fall in drogue until 750m.
            if (max_altitude >= 30.0f && alt_agl < (max_altitude - 10.0f)) {
                if (alt_agl <= PARAFOIL_DEPLOY_ALT) {
                    transitionTo(FlightState::PARAFOIL_DEPLOYMENT, now_ms);
                } else {
                    transitionTo(FlightState::DROGUE_DESCENT, now_ms);
                }
            }
            break;

        case FlightState::DROGUE_DESCENT:
            // Fast drogue descent (17 m/s): burnwire starts pulsing at 750m AGL
            if (alt_agl <= PARAFOIL_DEPLOY_ALT) {
                transitionTo(FlightState::PARAFOIL_DEPLOYMENT, now_ms);
            }
            break;

        case FlightState::PARAFOIL_DEPLOYMENT:
            // 1. Burnwire pulses until descent velocity decreases by at least 3.0 m/s
            if (burnwire_active) {
                float vel_drop = deploy_trigger_vz - filtered_descent_rate;
                if (vel_drop >= BURNWIRE_VEL_DROP_THRESH) {
                    burnwire_active = false; // Deceleration achieved! Reefing line cut & canopy open
                } else if (now_ms - burnwire_start_ms >= BURNWIRE_MAX_DURATION_MS) {
                    burnwire_active = false; // Safety timeout cutoff
                }
            }

            // 2. Start up servos and enter precision guidance when passing below 500m AGL
            if (alt_agl <= SERVO_START_ALT || alt_agl <= TOUCHDOWN_ALT) {
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
