#pragma once

#include "nav_types.h"
#include "config.h"

class FlightStateMachine {
public:
    FlightStateMachine();

    void reset();
    void update(float alt_agl, float heading, const LocalPoint& pos, const LocalPoint& target, float dt, uint32_t now_ms);
    void forceState(FlightState next_state, uint32_t now_ms);
    void tickTimers(uint32_t now_ms);

    FlightState getState() const { return current_state; }
    const char* getStateName() const { return getFlightStateName(current_state); }

    bool isSteeringEnabled() const;
    bool isFlywheelActive() const;
    bool isBurnwireActive() const { return burnwire_active; }
    bool isBuzzerActive() const { return buzzer_active; }

    float getMaxAltitude() const { return max_altitude; }
    float getBreakoutAltitude() const { return last_breakout_alt; }

private:
    FlightState current_state;
    float max_altitude;
    float last_altitude;
    uint32_t state_entry_time_ms;
    uint32_t still_start_time_ms;
    uint32_t burnwire_start_ms;

    bool burnwire_active;
    bool buzzer_active;
    float last_breakout_alt;

    void transitionTo(FlightState next_state, uint32_t now_ms);
};
