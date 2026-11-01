#pragma once

#include <stdint.h>
#include "nav_types.h"

struct EKFStateEstimate {
    float x;                     // East position [m]
    float y;                     // North position [m]
    float altitude;              // Altitude AGL [m]
    float vx;                    // Ground velocity East [m/s]
    float vy;                    // Ground velocity North [m/s]
    float vz;                    // Vertical velocity [m/s]
    float heading_deg;           // Heading [0, 360)
    float heading_rate_deg_s;    // Turn rate [deg/s]
    float sigma_x;
    float sigma_y;
    float sigma_alt;
    float sigma_heading_deg;
};

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    void reset(
        float init_x = 0.0f,
        float init_y = 0.0f,
        float init_alt = 0.0f,
        float init_heading_deg = 0.0f
    );

    void setKinematics(float forward_speed, float descent_rate);
    void setWindEstimate(float wind_x, float wind_y);

    void predict(float dt, float steering_cmd);

    bool correctGPS(float meas_x, float meas_y, float meas_vx = 0.0f, float meas_vy = 0.0f, bool has_vel = false);
    bool correctBaro(float meas_alt);
    bool correctHeading(float meas_heading_deg);

    float getX() const { return x[0]; }
    float getY() const { return x[1]; }
    float getAltitude() const { return x[2]; }
    float getHeadingDeg() const;

private:
    float x[5]; // State: [x, y, z, psi, w_rate]
    float P[5]; // Diagonal state variance: [Px, Py, Pz, Ppsi, Pw]

    float current_va;
    float current_vz;
    float wind_x;
    float wind_y;
};
