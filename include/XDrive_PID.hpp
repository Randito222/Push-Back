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
// Heading Modes
// =============================
enum class HeadingMode {
  ABSOLUTE,
  HOLD,
  FACE_TARGET
};

// =============================
// X-Drive PID (encoder-only pseudo-odom)
// NOTE: In the provided cpp this is delegated to odom drive for simplicity.
// =============================
void DriveToPoint_PID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxSpeed    = 110,
    int    timeout_ms  = 3000,
    double slewRateV   = 8
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
    double slewRateV   = 8
);

// Overload for absolute heading mode
inline void DriveToPoint_OdomPID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxSpeed    = 110,
    int    timeout_ms  = 3000,
    double slewRateV   = 8
) {
  DriveToPoint_OdomPID(targetX_in, targetY_in, targetHeading_deg,
                       HeadingMode::ABSOLUTE, maxSpeed, timeout_ms, slewRateV);
}
