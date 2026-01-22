#pragma once
#include <cmath>

// =============================
// Utility Helpers
// =============================
double clamp(double v, double lo, double hi);
double Myslew(double target, double current, double maxDelta);
void stopDrive();

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
void DriveToPoint_OdomPID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxVolt      = 12000,
    int    timeout_ms   = 3000,
    double slewRateV    = 300
<<<<<<< HEAD
);
=======
);
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
