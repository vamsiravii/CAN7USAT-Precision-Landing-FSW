#pragma once

#include "nav_types.h"
#include "config.h"

// ----------------------------------------------------------------------------
// Wind Estimator (Exponential Moving Average)
// ----------------------------------------------------------------------------
class WindEstimator {
public:
    WindEstimator(float alpha = WIND_EMA_ALPHA);
    void reset();
    void update(float raw_wx, float raw_wy);

    float getWindX() const { return wind_x; }
    float getWindY() const { return wind_y; }

private:
    float alpha;
    float wind_x;
    float wind_y;
    bool initialized;
};

// ----------------------------------------------------------------------------
// Radian-Domain PID Controller
// ----------------------------------------------------------------------------
class PIDController {
public:
    PIDController(
        float kp = PID_KP,
        float ki = PID_KI,
        float kd = PID_KD,
        float out_min = -1.0f,
        float out_max = 1.0f,
        float int_min = -PID_INTEGRAL_LIMIT,
        float int_max = PID_INTEGRAL_LIMIT
    );

    void reset();
    float update(float error_rad, float dt);

private:
    float kp, ki, kd;
    float output_min, output_max;
    float integral_min, integral_max;
    float integral;
    float previous_error;
    bool initialized;
};

// ----------------------------------------------------------------------------
// Sequential Waypoint Manager
// ----------------------------------------------------------------------------
class WaypointManager {
public:
    WaypointManager();
    void init(const LocalPoint& start, const LocalPoint& target, int num_pts = NUM_WAYPOINTS, float radius = WAYPOINT_ACCEPTANCE_RAD);
    void reset();
    bool update(const LocalPoint& current_pos);
    void skipToFinalTarget();

    LocalPoint getCurrentWaypoint() const;
    LocalPoint getTarget() const { return target_pt; }
    int getCurrentIndex() const { return current_idx; }
    int getTotalWaypoints() const { return total_pts; }
    bool isComplete() const { return current_idx >= total_pts; }

private:
    LocalPoint waypoints[NUM_WAYPOINTS];
    uint8_t total_pts;
    uint8_t current_idx;
    float acceptance_radius;
    LocalPoint target_pt;
};

// ----------------------------------------------------------------------------
// Guidance Controller (En-Route Waypoint Navigation)
// ----------------------------------------------------------------------------
class GuidanceController {
public:
    GuidanceController(WaypointManager* wm, PIDController* pid);
    void setWind(float wx, float wy) { wind_x = wx; wind_y = wy; }
    void reset();
    GuidanceOutput update(const LocalPoint& position, float heading, float altitude, float dt);

private:
    WaypointManager* waypoint_manager;
    PIDController* pid;
    float wind_x;
    float wind_y;
};

// ----------------------------------------------------------------------------
// Loiter Holding Orbit Guidance (Terminal Energy Dissipation)
// ----------------------------------------------------------------------------
class LoiterGuidance {
public:
    LoiterGuidance();
    void setCenter(const LocalPoint& c) { center = c; }
    void setWind(float wx, float wy) { wind_x = wx; wind_y = wy; }
    void setDirection(LoiterDirection dir) { direction = dir; }

    float update(const LocalPoint& position, float current_heading);

    float getRadialDistance() const { return radial_dist; }
    float getRadialError() const { return radial_err; }
    LoiterPhase getPhase() const { return phase; }
    float getDesiredHeading() const { return desired_hdg; }

private:
    LocalPoint center;
    float radius;
    LoiterDirection direction;
    float capture_tolerance;
    float forward_speed;
    float max_turn_rate;
    float wind_x;
    float wind_y;

    float radial_dist;
    float radial_err;
    float desired_hdg;
    LoiterPhase phase;
};

// ----------------------------------------------------------------------------
// Energy Check & Dynamic Breakout Guidance
// ----------------------------------------------------------------------------
bool shouldEnterTerminalGuidance(
    float altitude,
    float target_distance,
    float wind_x,
    float wind_y,
    float desired_course_deg
);

BreakoutGeometry calculateBreakoutGeometry(
    const LocalPoint& position,
    float heading,
    const LocalPoint& target,
    float wind_x,
    float wind_y
);
