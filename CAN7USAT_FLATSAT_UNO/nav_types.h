#pragma once

#include <stdint.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Local Cartesian and GPS Coordinates
// ----------------------------------------------------------------------------
struct LocalPoint {
    float x; // East [m]
    float y; // North [m]

    LocalPoint() = default;
    LocalPoint(float _x, float _y) : x(_x), y(_y) {}
};

struct GPSPoint {
    double lat; // Decimal degrees
    double lon;

    GPSPoint() = default;
    GPSPoint(double _lat, double _lon) : lat(_lat), lon(_lon) {}
};

// ----------------------------------------------------------------------------
// Guidance & Control Enums
// ----------------------------------------------------------------------------
enum class LoiterDirection {
    CLOCKWISE = 1,
    COUNTER_CLOCKWISE = -1
};

enum class LoiterPhase {
    ORBIT_CAPTURE,
    ORBIT_TRACKING
};

// ----------------------------------------------------------------------------
// Mission Flight Phases
// ----------------------------------------------------------------------------
enum class FlightState : uint8_t {
    BOOT = 0,
    LAUNCH_PAD = 1,
    ASCENT = 2,
    DROGUE_DESCENT = 3,         // Reefed parachute descent at ~17 m/s (1000m -> 600m)
    PARAFOIL_DEPLOYMENT = 4,    // 600m burnwire disreefing & 2s inflation
    PRECISION_GUIDANCE = 5,     // Waypoint progression to target area
    TERMINAL_GUIDANCE = 6,      // 30m loiter holding orbit over target
    FINAL_APPROACH = 7,         // Dynamic breakout direct glide to target
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
// Guidance Results
// ----------------------------------------------------------------------------
struct WindCompResult {
    float heading_deg;
    float crab_angle_deg;
    float crosswind_component;
    float along_track_wind;
    float ground_speed_along_track;
    bool feasible;
};

struct GuidanceOutput {
    float steering;                   // Commanded steering [-1.0, +1.0]
    float desired_bearing;            // Course to waypoint [deg]
    float heading_error;              // Signed error [deg]
    float distance_to_waypoint;       // Meters
    int waypoint_number;              // 1-indexed
    bool waypoint_reached;
    bool target_reached;
    bool final_approach;
    float desired_heading;            // Crab-corrected heading [deg]
    float crab_angle;
    bool wind_feasible;
};

struct BreakoutGeometry {
    float radial_distance;
    float target_bearing;
    float heading_error;
    float turn_angle;
    float turn_direction;             // +1.0 for CW, -1.0 for CCW
    float turn_radius;
    float turn_duration;
    float turn_arc_length;
    LocalPoint turn_endpoint;
    float straight_distance;
    float required_path_length;
    float glide_ratio;
    float minimum_required_altitude;
    float breakout_altitude;
    bool feasible;
};
