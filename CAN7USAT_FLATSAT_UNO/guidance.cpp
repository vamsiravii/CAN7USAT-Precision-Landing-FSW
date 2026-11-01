#include "guidance.h"
#include "nav_math.h"
#include "config.h"

// ============================================================================
// Wind Estimator
// ============================================================================
WindEstimator::WindEstimator(float a) : alpha(a) {}

void WindEstimator::reset() {
    wind_x = 0.0f;
    wind_y = 0.0f;
    initialized = false;
}

void WindEstimator::update(float raw_wx, float raw_wy) {
    if (!initialized) {
        wind_x = raw_wx;
        wind_y = raw_wy;
        initialized = true;
    } else {
        wind_x = alpha * raw_wx + (1.0f - alpha) * wind_x;
        wind_y = alpha * raw_wy + (1.0f - alpha) * wind_y;
    }
}

// ============================================================================
// PID Controller (Radian-Domain Input)
// ============================================================================
PIDController::PIDController(
    float _kp, float _ki, float _kd,
    float out_min, float out_max,
    float int_min, float int_max
) : kp(_kp), ki(_ki), kd(_kd),
    output_min(out_min), output_max(out_max),
    integral_min(int_min), integral_max(int_max) {}

void PIDController::reset() {
    integral = 0.0f;
    previous_error = 0.0f;
    initialized = false;
}

__attribute__((noinline))
float PIDController::update(float error_rad, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Proportional
    float p_term = kp * error_rad;

    // Integral with anti-windup clamping
    integral += error_rad * dt;
    if (integral < integral_min) integral = integral_min;
    if (integral > integral_max) integral = integral_max;
    float i_term = ki * integral;

    // Derivative
    float d_term = 0.0f;
    if (!initialized) {
        initialized = true;
    } else {
        d_term = kd * ((error_rad - previous_error) / dt);
    }
    previous_error = error_rad;

    float output = p_term + i_term + d_term;
    if (output < output_min) output = output_min;
    if (output > output_max) output = output_max;

    return output;
}

// ============================================================================
// Waypoint Manager
// ============================================================================
WaypointManager::WaypointManager() : acceptance_radius(WAYPOINT_ACCEPTANCE_RAD) {}

void WaypointManager::init(const LocalPoint& start, const LocalPoint& target, int num_pts, float radius) {
    total_pts = (num_pts > NUM_WAYPOINTS) ? NUM_WAYPOINTS : (uint8_t)num_pts;
    if (total_pts < 1) total_pts = 1;
    current_idx = 0;
    acceptance_radius = radius;
    target_pt = target;

    for (int i = 1; i <= total_pts; i++) {
        float fraction = (float)i / (float)total_pts;
        waypoints[i - 1].x = start.x + fraction * (target.x - start.x);
        waypoints[i - 1].y = start.y + fraction * (target.y - start.y);
    }
}

void WaypointManager::reset() {
    current_idx = 0;
}

bool WaypointManager::update(const LocalPoint& current_pos) {
    if (isComplete()) return false;

    bool changed = false;
    while (!isComplete()) {
        LocalPoint wp = waypoints[current_idx];
        float dist = distance_xy(current_pos, wp);
        if (dist <= acceptance_radius) {
            current_idx++;
            changed = true;
            continue;
        }

        // Standard passing plane check: advance if vehicle has progressed past
        // the waypoint along the track vector toward the target
        float v_x = target_pt.x - wp.x;
        float v_y = target_pt.y - wp.y;
        float p_x = current_pos.x - wp.x;
        float p_y = current_pos.y - wp.y;
        float dot = p_x * v_x + p_y * v_y;
        if (dot > 0.0f) {
            current_idx++;
            changed = true;
            continue;
        }
        break;
    }
    return changed;
}

void WaypointManager::skipToFinalTarget() {
    current_idx = total_pts;
}

LocalPoint WaypointManager::getCurrentWaypoint() const {
    if (isComplete()) return target_pt;
    return waypoints[current_idx];
}

// ============================================================================
// Guidance Controller
// ============================================================================
GuidanceController::GuidanceController(WaypointManager* wm, PIDController* p)
    : waypoint_manager(wm), pid(p), wind_x(0.0f), wind_y(0.0f) {}

void GuidanceController::reset() {
    if (waypoint_manager) waypoint_manager->reset();
    if (pid) pid->reset();
}

__attribute__((noinline))
GuidanceOutput GuidanceController::update(const LocalPoint& position, float heading, float /*altitude*/, float dt) {
    GuidanceOutput out = {};
    if (!waypoint_manager || !pid) return out;

    if (waypoint_manager->update(position)) {
        pid->reset();
    }

    LocalPoint active_wp = waypoint_manager->getCurrentWaypoint();
    float desired_course = bearing(position, active_wp);
    WindCompResult comp = calculate_wind_corrected_heading(desired_course, FORWARD_AIRSPEED, wind_x, wind_y);
    float err_deg = heading_error(heading, comp.heading_deg);
    out.steering = pid->update(deg2rad(err_deg), dt);
    return out;
}

// ============================================================================
// Loiter Guidance
// ============================================================================
LoiterGuidance::LoiterGuidance()
    : center(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH),
      radius(LOITER_RADIUS),
      direction(LoiterDirection::CLOCKWISE),
      capture_tolerance(LOITER_CAPTURE_TOL),
      forward_speed(FORWARD_AIRSPEED),
      max_turn_rate(MAX_TURN_RATE_DEG_S) {}

__attribute__((noinline))
float LoiterGuidance::update(const LocalPoint& position, float current_heading) {
    radial_dist = distance_xy(center, position);
    radial_err = radial_dist - radius;

    float dir_sign = (direction == LoiterDirection::CLOCKWISE) ? 1.0f : -1.0f;
    bool is_tracking = fabsf(radial_err) <= capture_tolerance;
    phase = is_tracking ? LoiterPhase::ORBIT_TRACKING : LoiterPhase::ORBIT_CAPTURE;

    float desired_course = 0.0f;
    float feedforward = 0.0f;

    if (is_tracking) {
        float pure_tangent = calculate_tangent_heading(center, position, direction);
        float corr_deg = rad2deg(atanf(radial_err / radius));
        if (corr_deg > 30.0f) corr_deg = 30.0f;
        if (corr_deg < -30.0f) corr_deg = -30.0f;

        desired_course = wrap_360(pure_tangent + dir_sign * corr_deg);
        float nominal_turn_rate = rad2deg(forward_speed / radius);
        feedforward = dir_sign * (nominal_turn_rate / max_turn_rate);
    } else if (radial_dist > radius + capture_tolerance) {
        float theta_to_center = bearing(position, center);
        float ratio = radius / radial_dist;
        if (ratio > 1.0f) ratio = 1.0f;
        float alpha_deg = rad2deg(ratio * (1.0f + 0.166667f * ratio * ratio));
        desired_course = wrap_360(theta_to_center - dir_sign * alpha_deg);
    } else if (radial_dist < 1e-4f) {
        desired_course = current_heading;
    } else {
        float rad_b = bearing(center, position);
        float ratio = radial_dist / radius;
        if (ratio > 1.0f) ratio = 1.0f;
        float gamma_deg = rad2deg(ratio * (1.0f + 0.166667f * ratio * ratio));
        desired_course = wrap_360(rad_b + dir_sign * gamma_deg);
    }

    WindCompResult comp = calculate_wind_corrected_heading(desired_course, forward_speed, wind_x, wind_y);
    desired_hdg = comp.heading_deg;

    float err = heading_error(current_heading, desired_hdg);
    float cmd = feedforward + (err / 45.0f);
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    return cmd;
}

// ============================================================================
// Energy Check
// ============================================================================
__attribute__((noinline))
bool shouldEnterTerminalGuidance(
    float altitude,
    float target_distance,
    float /*wind_x*/,
    float /*wind_y*/,
    float /*desired_course_deg*/
) {
    if (altitude <= 0.0f || target_distance <= 0.0f || target_distance > TERMINAL_CAPTURE_RADIUS) return false;
    float glide_range = (FORWARD_AIRSPEED / PARAFOIL_DESCENT_RATE) * altitude;
    return (glide_range >= (target_distance + TERMINAL_MARGIN));
}

// ============================================================================
// Breakout Geometry Solver
// ============================================================================
__attribute__((noinline))
BreakoutGeometry calculateBreakoutGeometry(
    const LocalPoint& position,
    float heading,
    const LocalPoint& target,
    float /*wind_x*/,
    float /*wind_y*/
) {
    BreakoutGeometry geom = {};
    float dist = distance_xy(position, target);
    float t_straight = dist / FORWARD_AIRSPEED;
    float t_turn = fabsf(heading_error(heading, bearing(position, target))) / MAX_TURN_RATE_DEG_S;
    geom.minimum_required_altitude = PARAFOIL_DESCENT_RATE * (t_straight + t_turn);
    geom.breakout_altitude = geom.minimum_required_altitude + BREAKOUT_ALT_MARGIN;
    return geom;
}
