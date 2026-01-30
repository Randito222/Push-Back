#include "XDrive_PID.hpp"
#include "OdomSet.hpp"
#include "main.h"
#include "subsystems.hpp"
#include <cmath>
#include <algorithm>

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
constexpr double GEAR_RATIO   = 1.0;
constexpr double PI = 3.141592653589793;

// =============================
// Utility
// =============================
double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

double Myslew(double target, double current, double step) {
  double diff = target - current;
  if (std::fabs(diff) <= step) return target;
  return current + (diff > 0 ? step : -step);
}



// =============================
// Encoder helpers (legacy DriveToPoint_PID)
// =============================
static double degToIn(double deg) {
  return (deg / 360.0) * PI * WHEEL_DIAM_IN / GEAR_RATIO;
}

static double avg(double a, double b) { return (a + b) * 0.5; }

static double flDeg() { return avg(FL1.get_position(), FL2.get_position()); }
static double frDeg() { return avg(FR1.get_position(), FR2.get_position()); }
static double blDeg() { return avg(BL1.get_position(), BL2.get_position()); }
static double brDeg() { return avg(BR1.get_position(), BR2.get_position()); }

static double xPos() {
  return degToIn((flDeg() - frDeg() - blDeg() + brDeg()) / 4.0);
}

static double yPos() {
  return degToIn((flDeg() + frDeg() + blDeg() + brDeg()) / 4.0);
}

// =============================
// IMU helpers (legacy DriveToPoint_PID)
// =============================
static double wrapDeg(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

static double imuHeading() {
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
// Legacy encoder-only point drive
// =============================
void DriveToPoint_PID(
    double targetX,
    double targetY,
    double targetHeading,
    int    maxSpeed,
    int    timeout_ms,
    double slewRateV
) {
  FL1.tare_position(); FL2.tare_position();
  FR1.tare_position(); FR2.tare_position();
  BL1.tare_position(); BL2.tare_position();
  BR1.tare_position(); BR2.tare_position();

  PIDTest xPID {10.0, 0.02, 40.0};
  PIDTest yPID {10.0, 0.02, 40.0};
  PIDTest turnPID {3.0, 0.01, 24.0};

  double fl = 0, fr = 0, bl = 0, br = 0;
  int settled = 0;
  int start = pros::millis();

  while (pros::millis() - start < timeout_ms) {
    double xErr = targetX - xPos();
    double yErr = targetY - yPos();
    double tErr = wrapDeg(targetHeading - imuHeading());

    if (std::fabs(xErr) < 0.5 &&
        std::fabs(yErr) < 0.5 &&
        std::fabs(tErr) < 1.0) {
      settled += 10;
      if (settled > 200) break;
    } else settled = 0;

    double xOut = clamp(xPID.step(xErr), -maxSpeed, maxSpeed);
    double yOut = clamp(yPID.step(yErr), -maxSpeed, maxSpeed);
    double tOut = clamp(turnPID.step(tErr), -maxSpeed, maxSpeed);

    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    fl = Myslew(tFL, fl, slewRateV);
    fr = Myslew(tFR, fr, slewRateV);
    bl = Myslew(tBL, bl, slewRateV);
    br = Myslew(tBR, br, slewRateV);

    FL1.move(fl); FL2.move(fl);
    FR1.move(fr); FR2.move(fr);
    BL1.move(bl); BL2.move(bl);
    BR1.move(br); BR2.move(br);

    pros::delay(10);
  }

  stopDrive();
}

// =============================
// Field-centric odom point drive
// =============================
void DriveToPoint_OdomPID(
    double targetX,
    double targetY,
    double targetHeadingDeg,
    HeadingMode headingMode,
    int    maxSpeed,
    int    timeout_ms,
    double slewRateV
) {
  PIDTest xPID    {10.0, 0.008, 30.0};
  PIDTest yPID    {10.0, 0.008, 30.0};
  PIDTest turnPID { 3.0, 0.00, 18.0};

  xPID.reset();
  yPID.reset();
  turnPID.reset();

  const double MIN_XY   = 8.0;
  const double MIN_TURN = 6.0;

  const double XY_ERR_GATE   = 0.6;
  const double TURN_ERR_GATE = 2.0;

  auto applyMin = [&](double out, double minv) -> double {
    if (std::fabs(out) < 1e-6) return 0.0;
    if (std::fabs(out) < minv) return (out > 0) ? minv : -minv;
    return out;
  };

  auto applyMinWithErrGate = [&](double out, double err, double minv, double gate) -> double {
    if (std::fabs(err) < gate) return 0.0;
    return applyMin(out, minv);
  };

  double fl = 0, fr = 0, bl = 0, br = 0;
  int settled = 0;
  int start = pros::millis();

  // For HOLD mode
  const double holdHeadingDeg = odomTheta * 180.0 / PI;

  while (pros::millis() - start < timeout_ms) {

    // =============================
    // FIELD ERRORS (inches)
    // =============================
    const double xErr = targetX - odomX;
    const double yErr = targetY - odomY;
    const double distErr = std::hypot(xErr, yErr);

    // =============================
    // Heading target (based on mode)
    // =============================
    const double headingDeg = odomTheta * 180.0 / PI;
    double headingTargetDeg = targetHeadingDeg;

    if (headingMode == HeadingMode::HOLD) {
      headingTargetDeg = holdHeadingDeg;
    } else if (headingMode == HeadingMode::FACE_TARGET) {
      // 0deg means +Y, so use atan2(x, y)
      headingTargetDeg = std::atan2(xErr, yErr) * 180.0 / PI;
    }

    double tErr = headingTargetDeg - headingDeg;
    while (tErr > 180) tErr -= 360;
    while (tErr < -180) tErr += 360;
    if (std::fabs(tErr) < 1.0) tErr = 0.0;

    // =============================
    // SETTLE CHECK
    // =============================
    if (distErr < 0.5 && std::fabs(tErr) < 0.7) {
      settled += 10;
      if (settled > 200) break;
    } else {
      settled = 0;
    }

    // =============================
    // FIELD -> ROBOT transform
    // robotX = strafe error, robotY = forward error
    // =============================
    const double sinH = std::sin(odomTheta);
    const double cosH = std::cos(odomTheta);

    const double robotX =  xErr * cosH + yErr * sinH;
    const double robotY = -xErr * sinH + yErr * cosH;

    // =============================
    // PID outputs (robot frame)
    // =============================
    double xOut = clamp(xPID.step(robotX), -maxSpeed, maxSpeed);
    double yOut = clamp(yPID.step(robotY), -maxSpeed, maxSpeed);

    double turnScale = clamp(1.0 - (distErr / 24.0), 0.25, 1.0);
    double tOut = clamp(turnPID.step(tErr) * turnScale, -maxSpeed, maxSpeed);

    // =============================
    // Minimum output with gating
    // =============================
    xOut = applyMinWithErrGate(xOut, robotX, MIN_XY, XY_ERR_GATE);
    yOut = applyMinWithErrGate(yOut, robotY, MIN_XY, XY_ERR_GATE);
    tOut = applyMinWithErrGate(tOut, tErr,   MIN_TURN, TURN_ERR_GATE);

    // =============================
    // X-DRIVE mixing
    // =============================
    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    // Normalize
    double maxMag = std::max({ std::fabs(tFL), std::fabs(tFR), std::fabs(tBL), std::fabs(tBR) });
    if (maxMag > maxSpeed) {
      double scale = (double)maxSpeed / maxMag;
      tFL *= scale; tFR *= scale; tBL *= scale; tBR *= scale;
    }

    // Slew
    fl = Myslew(tFL, fl, slewRateV);
    fr = Myslew(tFR, fr, slewRateV);
    bl = Myslew(tBL, bl, slewRateV);
    br = Myslew(tBR, br, slewRateV);

    // Apply
    FL1.move((int)fl); FL2.move((int)fl);
    FR1.move((int)fr); FR2.move((int)fr);
    BL1.move((int)bl); BL2.move((int)bl);
    BR1.move((int)br); BR2.move((int)br);

    // Debug
    pros::lcd::print(4, "FieldErr x%.1f y%.1f d%.1f", xErr, yErr, distErr);
    pros::lcd::print(5, "RobotErr rx%.1f ry%.1f", robotX, robotY);
    pros::lcd::print(6, "Head %.1f targ %.1f tErr %.1f", headingDeg, headingTargetDeg, tErr);
    pros::lcd::print(7, "Out x%.1f y%.1f t%.1f", xOut, yOut, tOut);

    pros::delay(10);
  }

  stopDrive();
}
