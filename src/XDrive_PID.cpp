#include "XDrive_PID.hpp"
#include "main.h"
#include "subsystems.hpp"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================
// Motor aliases
// =============================
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

// =============================
// Constants
// =============================
constexpr double WHEEL_DIAM_IN = 3.25;
constexpr double GEAR_RATIO    = 1.0;

// =============================
// Utility (static to avoid link collisions)
// =============================
static double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

double Myslew(double target, double current, double step) {
  double diff = target - current;
  if (std::fabs(diff) <= step) return target;
  return current + (diff > 0 ? step : -step);
}

// =============================
// PID
// =============================
struct PIDTest {
  double kP = 0, kI = 0, kD = 0;
  double integral = 0, prevErr = 0;
  double iLimit = 50;
  double iZone  = 5;   // only integrate when |err| < iZone

  double step(double err) {
    if (std::fabs(err) < iZone) integral += err;
    else integral = 0;

    integral = clampd(integral, -iLimit, iLimit);

    double deriv = err - prevErr;
    prevErr = err;

    return kP * err + kI * integral + kD * deriv;
  }

  void reset() { integral = 0; prevErr = 0; }
};

// =============================
// Encoder helpers (pseudo odom)
// =============================
static double degToIn(double deg) {
  // motor degrees -> inches
  return (deg / 360.0) * (M_PI * WHEEL_DIAM_IN) / GEAR_RATIO;
}

static double avg(double a, double b) { return (a + b) * 0.5; }

static double flDeg() { return avg(FL1.get_position(), FL2.get_position()); }
static double frDeg() { return avg(FR1.get_position(), FR2.get_position()); }
static double blDeg() { return avg(BL1.get_position(), BL2.get_position()); }
static double brDeg() { return avg(BR1.get_position(), BR2.get_position()); }

// Pseudo-odometry (encoder only)
static double xPos() {
  return degToIn((flDeg() - frDeg() - blDeg() + brDeg()) / 4.0);
}

static double yPos() {
  return degToIn((flDeg() + frDeg() + blDeg() + brDeg()) / 4.0);
}

// =============================
// IMU heading helpers
// =============================
static double wrapDeg(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

static double imuHeadingDeg() {
  return wrapDeg(IMU.get_rotation());
}

// =============================
// Stop
// =============================
void stopDrive() {
  FL1.move(0); FL2.move(0);
  FR1.move(0); FR2.move(0);
  BL1.move(0); BL2.move(0);
  BR1.move(0); BR2.move(0);
}

// =============================
// BASIC PID (encoder+IMU, NOT field-centric)
// =============================
void DriveToPoint_PID(
  double targetX,
  double targetY,
  double targetHeadingDeg,
  int    maxSpeed,
  int    timeout_ms,
  double slewRateV
) {
  // Reset encoders
  FL1.tare_position(); FL2.tare_position();
  FR1.tare_position(); FR2.tare_position();
  BL1.tare_position(); BL2.tare_position();
  BR1.tare_position(); BR2.tare_position();

  // PID tuning (starter values)
  PIDTest xPID    {10.0, 0.02, 40.0};
  PIDTest yPID    {10.0, 0.02, 40.0};
  PIDTest turnPID { 3.0, 0.01, 24.0};

  double fl = 0, fr = 0, bl = 0, br = 0;
  int settled = 0;
  int start = pros::millis();

  while (pros::millis() - start < timeout_ms) {
    double xErr = targetX - xPos();
    double yErr = targetY - yPos();
    double tErr = wrapDeg(targetHeadingDeg - imuHeadingDeg());

    if (std::fabs(xErr) < 0.5 && std::fabs(yErr) < 0.5 && std::fabs(tErr) < 1.0) {
      settled += 10;
      if (settled > 200) break;
    } else settled = 0;

    double xOut = clampd(xPID.step(xErr), -maxSpeed, maxSpeed);
    double yOut = clampd(yPID.step(yErr), -maxSpeed, maxSpeed);
    double tOut = clampd(turnPID.step(tErr), -maxSpeed, maxSpeed);

    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    fl = Myslew(tFL, fl, slewRateV);
    fr = Myslew(tFR, fr, slewRateV);
    bl = Myslew(tBL, bl, slewRateV);
    br = Myslew(tBR, br, slewRateV);

    FL1.move((int)fl); FL2.move((int)fl);
    FR1.move((int)fr); FR2.move((int)fr);
    BL1.move((int)bl); BL2.move((int)bl);
    BR1.move((int)br); BR2.move((int)br);

    pros::delay(10);
  }

  stopDrive();
}

// =============================
// ODOM FIELD-CENTRIC PID (THIS IS THE ONE YOU WANT)
// Drives to (targetX,targetY) in FIELD coords regardless of facing.
// If robot is 180° turned, it naturally drives "backwards" to the same point.
// =============================
void DriveToPoint_OdomPID(
  double targetX,
  double targetY,
  double targetHeadingDeg,
  int    maxSpeed,     // move() units: 0..127
  int    timeout_ms,
  double slewRateV     // move() units per loop
) {
  // =============================
  // PID tuning (robot frame errors: inches, heading: deg)
  // =============================
  PIDTest xPID    {10.0, 0.008, 30.0}; // strafe
  PIDTest yPID    {10.0, 0.008, 30.0}; // forward
  PIDTest turnPID { 3.0, 0.000, 18.0}; // heading

  xPID.reset();
  yPID.reset();
  turnPID.reset();

  // =============================
  // Minimum output to break static friction (only when far enough)
  // =============================
  const double MIN_XY   = 8.0;
  const double MIN_TURN = 6.0;

  // "Close enough" gates (don’t force min near target)
  const double XY_GATE_IN   = 0.75; // inches
  const double TURN_GATE_DEG = 2.0; // degrees

  auto applyMin = [&](double out, double minv) -> double {
    if (std::fabs(out) < 1e-6) return 0.0;
    if (std::fabs(out) < minv) return (out > 0) ? minv : -minv;
    return out;
  };

  auto applyMinIfFar = [&](double out, double err, double minv, double gate) -> double {
    // If we’re close, do NOT force a minimum (prevents oscillation + stopping short)
    if (std::fabs(err) < gate) return out;
    return applyMin(out, minv);
  };

  // =============================
  // Loop state
  // =============================
  double fl = 0, fr = 0, bl = 0, br = 0;
  int settled = 0;
  int start = pros::millis();

  while (pros::millis() - start < timeout_ms) {

    // =============================
    // FIELD ERRORS (inches)
    // =============================
    double xErr = targetX - odomX; // field right+
    double yErr = targetY - odomY; // field forward+

    double distErr = std::hypot(xErr, yErr);

    // =============================
    // Heading error (deg, wrapped)
    // =============================
    double headingDeg = odomTheta * 180.0 / M_PI;
    double tErr = targetHeadingDeg - headingDeg;
    while (tErr > 180) tErr -= 360;
    while (tErr < -180) tErr += 360;

    // tiny deadband
    if (std::fabs(tErr) < 0.75) tErr = 0;

    // =============================
    // SETTLE CHECK
    // =============================
    if (distErr < 0.5 && std::fabs(tErr) < 1.0) {
      settled += 10;
      if (settled > 200) break;
    } else {
      settled = 0;
    }

    // =============================
    // FIELD -> ROBOT transform
    // robotX = strafe error, robotY = forward error
    // heading 0 rad = +Y (forward)
    // =============================
    double sinH = std::sin(odomTheta);
    double cosH = std::cos(odomTheta);

    double robotX =  xErr * cosH + yErr * sinH;   // strafe
    double robotY = -xErr * sinH + yErr * cosH;   // forward

    // =============================
    // PID outputs (robot frame)
    // =============================
    double xOut = clampd(xPID.step(robotX), -maxSpeed, maxSpeed);
    double yOut = clampd(yPID.step(robotY), -maxSpeed, maxSpeed);

    // Turning scale (optional): reduce turn when far to avoid spiraling
    double turnScale = clampd(1.0 - (distErr / 24.0), 0.25, 1.0);
    double tOut = clampd(turnPID.step(tErr) * turnScale, -maxSpeed, maxSpeed);

    // =============================
    // Apply minimum outputs ONLY when far enough
    // =============================
    xOut = applyMinIfFar(xOut, robotX, MIN_XY,   XY_GATE_IN);
    yOut = applyMinIfFar(yOut, robotY, MIN_XY,   XY_GATE_IN);
    tOut = applyMinIfFar(tOut, tErr,   MIN_TURN, TURN_GATE_DEG);

    // Optional output deadband to reduce chatter
    if (std::fabs(xOut) < 1.0) xOut = 0;
    if (std::fabs(yOut) < 1.0) yOut = 0;
    if (std::fabs(tOut) < 1.0) tOut = 0;

    // =============================
    // X-DRIVE mixing
    // =============================
    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    // =============================
    // Normalize so max wheel <= maxSpeed
    // =============================
    double maxMag = std::max({ std::fabs(tFL), std::fabs(tFR), std::fabs(tBL), std::fabs(tBR) });
    if (maxMag > maxSpeed) {
      double scale = (double)maxSpeed / maxMag;
      tFL *= scale; tFR *= scale; tBL *= scale; tBR *= scale;
    }

    // =============================
    // Slew rate limit
    // =============================
    fl = Myslew(tFL, fl, slewRateV);
    fr = Myslew(tFR, fr, slewRateV);
    bl = Myslew(tBL, bl, slewRateV);
    br = Myslew(tBR, br, slewRateV);

    // =============================
    // Apply to motors
    // =============================
    FL1.move((int)fl); FL2.move((int)fl);
    FR1.move((int)fr); FR2.move((int)fr);
    BL1.move((int)bl); BL2.move((int)bl);
    BR1.move((int)br); BR2.move((int)br);

    // =============================
    // Debug
    // =============================
    pros::lcd::print(4, "FieldErr x%.1f y%.1f d%.1f", xErr, yErr, distErr);
    pros::lcd::print(5, "RobotErr rx%.1f ry%.1f", robotX, robotY);
    pros::lcd::print(6, "Head %.1f tErr %.1f", headingDeg, tErr);
    pros::lcd::print(7, "Out x%.1f y%.1f t%.1f", xOut, yOut, tOut);

    pros::delay(10);
  }

  stopDrive();
}
