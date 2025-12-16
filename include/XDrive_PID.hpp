#pragma once
#include <cmath>

// =============================
// Utility Helpers
// =============================
double clamp(double v, double lo, double hi);
double slewRate(double target, double current, double maxDelta);
void StopBase();

// =============================
// X-Drive PID
// =============================
void DriveToPoint_PID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxVolt      = 12000,
    int    timeout_ms   = 3000,
    double slewRateV    = 300
);

// =============================
// X-Drive Odometry PID
// =============================
