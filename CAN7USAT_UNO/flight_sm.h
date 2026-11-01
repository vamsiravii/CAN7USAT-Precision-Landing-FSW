#pragma once

#include "nav_types.h"
#include "config.h"

// ============================================================================
// Mission sequencing: pad -> drone ascent -> release -> drogue -> burnwire
// (line parts at 600 m) -> canopy inflation -> guidance -> touchdown.
// Guidance sub-states (PRECISION / TERMINAL / FINAL) are requested by the
// guidance layer through requestGuidanceState().
// ============================================================================
class FlightStateMachine {
public:
    FlightStateMachine();

    void reset();
    void update(float alt_agl, float climb_rate, uint32_t now_ms);
    void requestGuidanceState(FlightState s, uint32_t now_ms);
    void forceState(FlightState next_state, uint32_t now_ms);
    void tickTimers(uint32_t now_ms);

    FlightState getState() const { return current_state; }
    const char* getStateName() const { return getFlightStateName(current_state); }

    bool isSteeringEnabled() const;
    bool isGuidanceState() const { return isSteeringEnabled(); }
    bool isFlywheelActive() const;
    bool isBurnwireActive() const { return burnwire_active; }
    bool isBuzzerActive() const { return buzzer_active; }
    bool isCanopyInflated() const { return canopy_inflated; }

    float getMaxAltitude() const { return max_altitude; }
    float getDeployTriggerAlt() const;
    float getDrogueRate() const { return drogue_rate; }
    float getCanopyOpenAlt() const { return canopy_open_alt; }
    float getBurnwireOnAlt() const { return burnwire_on_alt; }

private:
    FlightState current_state;
    float max_altitude;
    uint32_t state_entry_time_ms;
    uint32_t still_start_time_ms;
    uint32_t burnwire_start_ms;
    uint32_t slow_sink_start_ms;
    uint32_t inflated_ms;

    float drogue_rate;
    float canopy_open_alt;
    float burnwire_on_alt;
    bool burnwire_active;
    bool buzzer_active;
    bool canopy_inflated;
    bool seen_fast_fall;

    void transitionTo(FlightState next_state, uint32_t now_ms, float alt_agl);
};
