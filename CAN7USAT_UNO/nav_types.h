#pragma once

#include <stdint.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Local Cartesian Coordinates (origin = launch-pad GNSS fix)
// ----------------------------------------------------------------------------
struct LocalPoint {
    float x; // East [m]
    float y; // North [m]

    LocalPoint() = default;
    LocalPoint(float _x, float _y) : x(_x), y(_y) {}
};

// ----------------------------------------------------------------------------
// Mission Flight Phases (numeric values are telemetered - do not renumber)
// ----------------------------------------------------------------------------
enum class FlightState : uint8_t {
    BOOT = 0,
    LAUNCH_PAD = 1,
    ASCENT = 2,                 // Carried up by the drone
    DROGUE_DESCENT = 3,         // Drogue only, ~16 m/s (release -> ~625 m)
    PARAFOIL_DEPLOYMENT = 4,    // Burnwire cut, canopy inflation & settling
    PRECISION_GUIDANCE = 5,     // Homing toward the target
    TERMINAL_GUIDANCE = 6,      // Energy management: loiter + downwind leg
    FINAL_APPROACH = 7,         // Final leg into the target (S-turns burn excess height)
    LANDING = 8,                // Ground contact / Touchdown
    FAILSAFE = 9                // Emergency recovery
};

inline const char* getFlightStateName(FlightState s) {
    switch (s) {
        case FlightState::BOOT:                 return "BOOT";
        case FlightState::LAUNCH_PAD:            return "LAUNCH_PAD";
        case FlightState::ASCENT:                return "ASCENT";
        case FlightState::DROGUE_DESCENT:        return "DROGUE_DESCENT";
        case FlightState::PARAFOIL_DEPLOYMENT:   return "PARAFOIL_DEPLOY";
        case FlightState::PRECISION_GUIDANCE:    return "PRECISION_GUIDE";
        case FlightState::TERMINAL_GUIDANCE:     return "TERMINAL_LOITER";
        case FlightState::FINAL_APPROACH:        return "FINAL_APPROACH";
        case FlightState::LANDING:               return "LANDED";
        case FlightState::FAILSAFE:              return "FAILSAFE";
        default:                                 return "UNKNOWN";
    }
}

// ----------------------------------------------------------------------------
// Wind-triangle result (crab angle for a desired ground course)
// ----------------------------------------------------------------------------
struct WindCompResult {
    float heading_deg;
    float crab_angle_deg;
    float crosswind_component;
    float along_track_wind;
    float ground_speed_along_track;
    bool feasible;
};

// ----------------------------------------------------------------------------
// One 10 Hz snapshot of the sensors, already converted to engineering units.
// Filled by the Arduino sketch (or by the SIL harness) and passed to GncCore.
// ----------------------------------------------------------------------------
struct GncSensors {
    uint32_t now_ms;

    bool baro_valid;
    float baro_alt_agl;      // Filtered barometric altitude above the launch pad [m]

    bool gyro_valid;
    float yaw_rate_dps;      // Mean yaw rate over the last tick, + = clockwise seen from above [deg/s]

    bool gnss_new;           // A new PVT solution arrived since the previous tick
    bool gnss_valid;         // 3D fix, enough satellites and a launch origin exists
    float gnss_x;            // East of launch origin [m]
    float gnss_y;            // North of launch origin [m]
    float gnss_vn;           // Ground velocity North [m/s]
    float gnss_ve;           // Ground velocity East [m/s]
    float gnss_vd;           // Velocity Down [m/s]
};

// ----------------------------------------------------------------------------
// Actuator demands produced by GncCore every tick.
// ----------------------------------------------------------------------------
struct GncOutputs {
    float steering;          // [-1, +1], + = right brake = turn right (clockwise)
    bool steering_enabled;
    bool burnwire;
    bool flywheel;
    bool buzzer;
};
