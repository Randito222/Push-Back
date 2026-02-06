#include <algorithm>
#include "main.h"
#include <cmath>

extern double odomX;
extern double odomY;
extern double odomTheta;

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline double wrapPi(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
static inline double wrapDeg(double a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}
static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double sgn(double v) { return (v > 0) - (v < 0); }

void driveToPoint_XDrive_PID(
    double targetX,
    double targetY,
    double targetHeadingDeg,
    int maxDrive,
    int maxTurn,
    int timeoutMs
) {
  const uint32_t start = pros::millis();

  // Stronger translation gains (yours are too low to overcome friction)
  const double kP_xy = 6.0;
  const double kD_xy = 18.0;

  // Heading hold (optional — set to 0 if you want translation-only while debugging)
  const double kP_h = 35.0;     // was 80
  const double kD_h = 120.0;    // was 200

  const double posTol  = 1.0;                 // inches
  const double headTol = 2.0 * M_PI / 180.0;  // rad

  const double MIN_XY = 10.0;  // bump this while debugging
  const double MIN_T = 4.0;     // was 8 (too punchy)

  const double kSlow = 6.0;
  const int    minDriveNear = 12; // not 25, but not 0 either

  const int loopMs = 10;
  const double dt = loopMs / 1000.0;

  double lastXErr = 0, lastYErr = 0, lastHErr = 0;

  const double targetH = wrapPi(targetHeadingDeg * M_PI / 180.0);

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    const double cx = odomX;
    const double cy = odomY;
    const double ch = odomTheta;

    const double fxErr = targetX - cx;
    const double fyErr = targetY - cy;
    const double dist  = std::hypot(fxErr, fyErr);

    const double c = std::cos(ch);
    const double s = std::sin(ch);

    const double xErr =  fxErr * c + fyErr * s;  // robot strafe
    const double yErr = -fxErr * s + fyErr * c;  // robot forward
    const double hErr = wrapPi(targetH - ch);

    // Exit (simple)
    if (dist < posTol && std::fabs(hErr) < headTol) break;

    const double dxErr = (xErr - lastXErr) / dt;
    const double dyErr = (yErr - lastYErr) / dt;
    const double dhErr = (hErr - lastHErr) / dt;

    lastXErr = xErr; lastYErr = yErr; lastHErr = hErr;

    double xOut = kP_xy * xErr + kD_xy * dxErr;
    double yOut = kP_xy * yErr + kD_xy * dyErr;
    double hOut = kP_h  * hErr + kD_h  * dhErr;

    // Dynamic max translation
    int dynMaxDrive = (int)std::round(std::min<double>(maxDrive, std::max<double>(minDriveNear, dist * kSlow)));

    xOut = clampd(xOut, -dynMaxDrive, dynMaxDrive);
    yOut = clampd(yOut, -dynMaxDrive, dynMaxDrive);
    hOut = clampd(hOut, -maxTurn, maxTurn);

    // Minimum translation power based on ERROR (prevents “jolt then stop”)
    if (std::fabs(xErr) > posTol && std::fabs(xOut) < MIN_XY) xOut = sgn(xErr) * MIN_XY;
    if (std::fabs(yErr) > posTol && std::fabs(yOut) < MIN_XY) yOut = sgn(yErr) * MIN_XY;
    if (std::fabs(hErr) > headTol && std::fabs(hOut) < MIN_T)  hOut = sgn(hErr) * MIN_T;

    // Mix
    double fl = yOut + xOut + hOut;
    double fr = yOut - xOut - hOut;
    double bl = yOut - xOut + hOut;
    double br = yOut + xOut - hOut;

    // Normalize
    const double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br), 127.0});
    fl = fl * 127.0 / maxMag;
    fr = fr * 127.0 / maxMag;
    bl = bl * 127.0 / maxMag;
    br = br * 127.0 / maxMag;

    setDrivePower((int)fl, (int)fr, (int)bl, (int)br);
    pros::delay(loopMs);
  }

  setDrivePower(0,0,0,0);
}



void turnToHeading_PID(double targetDeg, int maxTurn, int timeoutMs) {
  const uint32_t start = pros::millis();

  const double kP = 2.2;
  const double kD = 8.0;

  const double tol = 1.5;
  const double MIN_T = 7.0;

  const int loopMs = 10;
  const double dt = loopMs / 1000.0;

  double lastErr = 0;

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    const double cur = IMU.get_rotation();
    const double err = wrapDeg(targetDeg - cur);

    if (std::fabs(err) < tol) break;

    double out = kP * err + kD * (err - lastErr) / dt;
    lastErr = err;

    out = clampd(out, -maxTurn, maxTurn);

    if (std::fabs(out) > 1 && std::fabs(err) > tol)
      out = sgn(out) * std::max(std::fabs(out), MIN_T);

    setDrivePower((int)out, (int)-out, (int)out, (int)-out);
    pros::delay(loopMs);
  }

  setDrivePower(0,0,0,0);
}

void driveFieldInches(
    double northIn,          // + = north (+Y), - = south
    double eastIn,           // + = east (+X),  - = west
    double holdHeadingDeg,   // keep facing this heading while moving
    int maxDrive,
    int maxTurn,
    int timeoutMs
) {
  const double targetX = odomX + eastIn;
  const double targetY = odomY + northIn;

  driveToPoint_XDrive_PID(targetX, targetY, holdHeadingDeg, maxDrive, maxTurn, timeoutMs);
}

