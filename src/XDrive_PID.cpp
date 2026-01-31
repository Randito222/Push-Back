#include "XDrive_PID.hpp"
#include "OdomSet.hpp"
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

static constexpr double RAD2DEG = 180.0 / M_PI;

// =============================
// Utility Helpers (match header)
// =============================
double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

double Myslew(double target, double current, double maxDelta) {
  double diff = target - current;
  if (std::fabs(diff) <= maxDelta) return target;
  return current + (diff > 0 ? maxDelta : -maxDelta);
}

void stopDrive() {
  FL1.move(0); FL2.move(0);
  FR1.move(0); FR2.move(0);
  BL1.move(0); BL2.move(0);
  BR1.move(0); BR2.move(0);
}

static double wrapDeg(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

// =============================================
// Encoder-only pseudo drive (fallback)
// =============================================
void DriveToPoint_PID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    int    maxSpeed,
    int    timeout_ms,
    double slewRateV
) {
  DriveToPoint_OdomPID(targetX_in, targetY_in, targetHeading_deg,
                       HeadingMode::ABSOLUTE, maxSpeed, timeout_ms, slewRateV);
}

// =============================================
// X-Drive Odometry PID (field-centric)
// Axis mapping requested:
//   targetX_in / odomX = forward/upfield
//   targetY_in / odomY = right
// Heading:
//   0 deg points along +X (forward), positive towards +Y (right)
// =============================================
void DriveToPoint_OdomPID(
    double targetX_in,
    double targetY_in,
    double targetHeading_deg,
    HeadingMode headingMode,
    int    maxSpeed,
    int    timeout_ms,
    double slewRateV
) {
  PIDTest fPID {10.0, 0.05, 30.0};   // forward
  PIDTest rPID {0.0,  0.0,  0.0};    // right (tune if needed)
  PIDTest turnPID {0.0, 0.0, 0.0};

  fPID.reset();
  rPID.reset();
  turnPID.reset();

  const double MIN_XY   = 8.0;
  const double MIN_TURN = 6.0;

  auto applyMin = [&](double v, double minv) -> double {
    if (std::fabs(v) < 1e-6) return 0.0;
    if (std::fabs(v) < minv) return (v > 0 ? minv : -minv);
    return v;
  };

  const double holdHeading = odomTheta * RAD2DEG;

  double fl=0, fr=0, bl=0, br=0;
  int start = pros::millis();
  int settled = 0;

  while (pros::millis() - start < timeout_ms) {
    // Field error in VEX GPS axes
    const double fErrF = targetX_in - odomX; // forward
    const double rErrF = targetY_in - odomY; // right
    const double dist  = std::hypot(fErrF, rErrF);

    const double headingDeg = odomTheta * RAD2DEG;

    // Heading target
    double headingTarget = targetHeading_deg;
    if (headingMode == HeadingMode::HOLD) {
      headingTarget = holdHeading;
    } else if (headingMode == HeadingMode::FACE_TARGET) {
      // 0° = +forward => atan2(right, forward)
      headingTarget = std::atan2(rErrF, fErrF) * RAD2DEG;
    }

    const double tErr = wrapDeg(headingTarget - headingDeg);

    if (dist < 1.0 && std::fabs(tErr) < 2.0) {
      settled += 10;
      if (settled > 200) break;
    } else settled = 0;

    // Field -> Robot transform (forward/right axes)
    const double sinH = std::sin(odomTheta);
    const double cosH = std::cos(odomTheta);

    const double robotForward =  fErrF * cosH + rErrF * sinH;
    const double robotRight   = -fErrF * sinH + rErrF * cosH;

    double fOut = fPID.step(robotForward);
    double rOut = rPID.step(robotRight);
    double tOut = turnPID.step(tErr);

    fOut = applyMin(clamp(fOut, -maxSpeed, maxSpeed), MIN_XY);
    rOut = applyMin(clamp(rOut, -maxSpeed, maxSpeed), MIN_XY);
    tOut = applyMin(clamp(tOut, -maxSpeed, maxSpeed), MIN_TURN);

    // Mix (y=forward, x=right)
    const double yOut = fOut;
    const double xOut = rOut;

    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    double m = std::max({std::fabs(tFL), std::fabs(tFR), std::fabs(tBL), std::fabs(tBR)});
    if (m > maxSpeed && m > 1e-6) {
      double s = (double)maxSpeed / m;
      tFL *= s; tFR *= s; tBL *= s; tBR *= s;
    }

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
