#pragma once

#include "nav_types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define EARTH_RADIUS_M 6371000.0f

// ----------------------------------------------------------------------------
// Angle Wrapping Utilities
// ----------------------------------------------------------------------------
inline float deg2rad(float deg) {
    return deg * ((float)M_PI / 180.0f);
}

inline float rad2deg(float rad) {
    return rad * (180.0f / (float)M_PI);
}

__attribute__((noinline))
inline float wrap_360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

__attribute__((noinline))
inline float wrap_180(float deg) {
    return wrap_360(deg + 180.0f) - 180.0f;
}

__attribute__((noinline))
inline float wrap_pi(float rad) {
    while (rad > (float)M_PI) rad -= 2.0f * (float)M_PI;
    while (rad < -(float)M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}

inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// ----------------------------------------------------------------------------
// 2D Distance & Bearing (bearing: 0 = North, 90 = East)
// ----------------------------------------------------------------------------
__attribute__((noinline))
inline float distance_xy(const LocalPoint& a, const LocalPoint& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

__attribute__((noinline))
inline float bearing(const LocalPoint& from, const LocalPoint& to) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    return wrap_360(rad2deg(atan2f(dx, dy)));
}

inline float heading_error(float current_heading, float desired_bearing) {
    return wrap_180(desired_bearing - current_heading);
}

// ----------------------------------------------------------------------------
// GNSS -> local East/North.  Inputs are the raw UBX integers (1e-7 deg), so
// the subtraction is exact even on AVR where double == float.
// cos_lat0 = cos(launch latitude), computed once when the origin is fixed.
// ----------------------------------------------------------------------------
inline LocalPoint e7_to_local(int32_t lat_e7, int32_t lon_e7,
                              int32_t lat0_e7, int32_t lon0_e7, float cos_lat0) {
    const float k = 1.0e-7f * ((float)M_PI / 180.0f) * EARTH_RADIUS_M;  // metres per 1e-7 deg
    float y = (float)(lat_e7 - lat0_e7) * k;
    float x = (float)(lon_e7 - lon0_e7) * k * cos_lat0;
    return LocalPoint(x, y);
}

// ----------------------------------------------------------------------------
// Wind triangle: heading needed to hold a ground course, and the resulting
// ground speed along that course.  Wind vector (wind_x, wind_y) is the
// direction the air MOVES toward (East, North components).
// ----------------------------------------------------------------------------
__attribute__((noinline))
inline WindCompResult calculate_wind_corrected_heading(
    float desired_course_deg,
    float airspeed,
    float wind_x,
    float wind_y
) {
    WindCompResult res;
    float chi_rad = deg2rad(desired_course_deg);

    // Crosswind component (to the right of the course): n = [cos(chi), -sin(chi)]
    float w_perp = wind_x * cosf(chi_rad) - wind_y * sinf(chi_rad);
    // Along-track wind: u = [sin(chi), cos(chi)]
    float w_parallel = wind_x * sinf(chi_rad) + wind_y * cosf(chi_rad);

    res.crosswind_component = w_perp;
    res.along_track_wind = w_parallel;

    if (airspeed < 1e-4f) {
        res.heading_deg = desired_course_deg;
        res.crab_angle_deg = 0.0f;
        res.ground_speed_along_track = 0.0f;
        res.feasible = false;
        return res;
    }

    res.feasible = (fabsf(w_perp) <= (airspeed + 1e-5f));

    if (res.feasible) {
        float ratio = clampf(-w_perp / airspeed, -1.0f, 1.0f);
        float delta_rad = asinf(ratio);
        res.crab_angle_deg = rad2deg(delta_rad);
        res.heading_deg = wrap_360(desired_course_deg + res.crab_angle_deg);
        res.ground_speed_along_track = airspeed * cosf(delta_rad) + w_parallel;
    } else {
        // Crosswind exceeds airspeed: point fully into the crosswind
        float delta_deg = (w_perp < 0.0f) ? 90.0f : -90.0f;
        res.crab_angle_deg = delta_deg;
        res.heading_deg = wrap_360(desired_course_deg + delta_deg);
        res.ground_speed_along_track = w_parallel;
    }
    return res;
}

// Ground speed achievable along a ground course (never below 0.3 m/s so that
// time-to-go predictions stay finite; callers treat <0.5 m/s as "no penetration").
inline float ground_speed_on_course(float course_deg, float airspeed, float wind_x, float wind_y) {
    WindCompResult r = calculate_wind_corrected_heading(course_deg, airspeed, wind_x, wind_y);
    return (r.ground_speed_along_track > 0.3f) ? r.ground_speed_along_track : 0.3f;
}
