#pragma once
#include <cmath>

// =============================
// Utility Helpers
// =============================
double clamp(double v, double lo, double hi);
double Myslew(double target, double current, double maxDelta);
void stopDrive();

// =============================
// PID
// =============================
struct PIDTest {
  double kP=0, kI=0, kD=0;
  double integral=0, prevErr=0;
  double iLimit=50;
  double iZone=5;

  double step(double err) {
    if (std::fabs(err) < iZone) integral += err;
    else integral = 0;

    integral = clamp(integral, -iLimit, iLimit);

    double deriv = err - prevErr;
    prevErr = err;

    return kP*err + kI*integral + kD*deriv;
  }

  void reset() { integral=0; prevErr=0; }
};

// =============================
// Heading Modes (for odom point drive)
// =============================
// ABSOLUTE:    turn to targetHeading_deg
// HOLD:        keep the robot's heading at the moment the function starts
// FACE_TARGET: turn to face the target point while driving (arrive facing the target direction)
enum class HeadingMode {
  ABSOLUTE,
  HOLD,
  FACE_TARGET
};

// =============================
// X-Drive PID (encoder-only pseudo-odom)
// =============================
void DriveToPoint_PID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxSpeed    = 110,
    int    timeout_ms  = 3000,
    double slewRateV   = 300
);

// =============================
// X-Drive Odometry PID (field-centric)
// =============================
void DriveToPoint_OdomPID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    HeadingMode headingMode,
    int    maxSpeed    = 110,
    int    timeout_ms  = 3000,
    double slewRateV   = 300
);

// Backwards-compatible overload (keeps your old calls working):
inline void DriveToPoint_OdomPID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxSpeed    = 110,
    int    timeout_ms  = 3000,
    double slewRateV   = 300
) {
  DriveToPoint_OdomPID(targetX_in, targetY_in, targetHeading_deg,
                       HeadingMode::ABSOLUTE, maxSpeed, timeout_ms, slewRateV);
}
