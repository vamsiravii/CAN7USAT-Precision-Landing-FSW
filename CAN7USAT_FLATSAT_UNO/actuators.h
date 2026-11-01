#pragma once

#include <stdint.h>
#include "config.h"
#include "nav_types.h"

class ActuatorManager {
public:
    ActuatorManager();

    void init();
    static bool isFlightStateSteeringPermitted(FlightState state);

    void applySteering(
        float steering,
        bool enabled,
        uint32_t now_ms,
        FlightState flight_state = FlightState::PRECISION_GUIDANCE
    );

    void update(uint32_t now_ms);
    void setDirectPulse(uint16_t left_us, uint16_t right_us);

    void setBurnwire(bool active);
    void updateBuzzer(bool active, uint32_t now_ms);
    void setFlywheel(bool active);

    float getRequestedSteeringEffort() const { return requested_steering_effort; }
    float getLeftBrake() const { return left_brake_norm; }
    float getRightBrake() const { return right_brake_norm; }
    int getLeftServoPulseUs() const { return (int)left_pulse_us; }
    int getRightServoPulseUs() const { return (int)right_pulse_us; }
    bool isBurnwirePinHigh() const { return burnwire_state; }
    bool isBuzzerPinHigh() const { return buzzer_state; }
    bool isFlywheelActive() const { return flywheel_state; }
    float getFlywheelRPM() const { return flywheel_state ? FLYWHEEL_TARGET_RPM : 0.0f; }
    int getFlywheelPulseUs() const { return (int)flywheel_pulse_us; }

private:
    float requested_steering_effort;
    float left_brake_norm;
    float right_brake_norm;
    uint16_t left_pulse_us;
    uint16_t right_pulse_us;
    uint16_t flywheel_pulse_us;
    uint32_t last_flywheel_ramp_ms;
    uint32_t last_buzzer_toggle_ms;
    bool burnwire_state;
    bool buzzer_state;
    bool flywheel_state;
};
