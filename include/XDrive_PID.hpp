#pragma once
#include <cmath>

// =============================
// Utility Helpers
// =============================
double clamp(double v, double lo, double hi);
double slewRate(double target, double current, double maxDelta);
void StopBase();

// =============================
// X-Drive Odometry PID
// =============================
void DriveToPoint_PID(
    double targetX,
    double targetY,
    double targetHeadingDeg,
    double maxSpeed,
    double slew
);
