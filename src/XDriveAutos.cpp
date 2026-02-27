#include <algorithm>
#include "main.h"
#include <cmath>

// =============================
// DRIVE TO POINT (ODOM X/Y TARGETS) — X-DRIVE
// Uses: odomX/odomY/odomTheta from pros::Task odomTask()
// Fixes included from our convo:
//  - Derivative is per-loop (no dt divide) to match your other code style (more stable)
//  - Soft-min outputs to overcome pod friction (MIN_XY / MIN_T)
//  - Dynamic translation clamp based on distance (prevents slamming near target)
//  - Cross-track “arc reduction”: if you're far off-line, reduce heading authority so it slides back instead of curving
//  - Settle-based exit (prevents "stop while still drifting")
//  - Brain LCD debug + optional USB printf stream
// =============================

extern double odomX;
extern double odomY;
extern double odomTheta;

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline double wrapPi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
static inline double wrapDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
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

  // -----------------------------
  // GAINS (start points)
  // -----------------------------
  // Translation PD (inches -> power)
  const double kP_xy = 6.0;
  const double kD_xy = 18.0;   // derivative is "per-loop change", not /dt

  // Heading PD (radians -> power)
  // NOTE: We run heading in radians for consistency with odomTheta.
  const double kP_h = 35.0;
  const double kD_h = 120.0;

  // -----------------------------
  // TOLERANCES + SETTLE
  // -----------------------------
  const double posTolIn   = 1.0;              // inches to target
  const double headTolRad = deg2rad(2.0);     // radians
  const int    settleReq  = 10;               // 10 * loopMs (~200ms)

  // -----------------------------
  // FRICTION / MIN OUTPUTS
  // -----------------------------
  const double MIN_XY = 10.0;  // minimum translation power when error exists
  const double MIN_T  = 4.0;   // minimum turn power when error exists

  // -----------------------------
  // DYNAMIC LIMITS (distance-based)
  // -----------------------------
  const double kSlow = 6.0;        // dist*6 => allowable power
  const int    minDriveNear = 12;  // keep some authority even near target

  // -----------------------------
  // ARC REDUCTION (prevents "curve to line")
  // -----------------------------
  // If you're far from the point, reduce heading power so translation can "snap"
  // then heading cleans up near the end.
  const double ARC_CROSS_IN = 1.0;    // inches
  const double ARC_H_MAX    = 12.0;   // cap heading output while correcting cross-track

  // -----------------------------
  // LOOP TIMING
  // -----------------------------
  const int loopMs = 20;

  // -----------------------------
  // STATE
  // -----------------------------
  double lastXErr = 0.0, lastYErr = 0.0, lastHErr = 0.0;
  int settleCount = 0;
  int lcdCounter  = 0;

  const double targetH = -wrapPi(deg2rad(targetHeadingDeg));

  while (true) {
    const uint32_t now = pros::millis();
    if ((int)(now - start) > timeoutMs) break;

    // Current pose from odomTask
    const double cx = odomX;
    const double cy = odomY;
    const double ch = odomTheta; // radians

    // Field-frame error to target
    const double fxErr = targetX - cx;
    const double fyErr = targetY - cy;
    const double dist  = std::hypot(fxErr, fyErr);

    // Convert FIELD error into ROBOT frame using odomTheta
    // robot X = strafe right, robot Y = forward
    const double c = std::cos(ch);
    const double s = std::sin(ch);

    const double xErr =  fxErr * c + fyErr * s;   // strafe error (in)
    const double yErr = -fxErr * s + fyErr * c;   // forward error (in)
    const double hErr = wrapPi(targetH - ch);     // heading error (rad)

    // -----------------------------
    // Settle-based exit
    // -----------------------------
    const bool posOK  = (dist < posTolIn);
    const bool headOK = (std::fabs(hErr) < headTolRad);

    if (posOK && headOK) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleReq) break;

    // -----------------------------
    // Derivatives (per-loop difference)
    // -----------------------------
    const double dxErr = (xErr - lastXErr);
    const double dyErr = (yErr - lastYErr);
    const double dhErr = (hErr - lastHErr);

    lastXErr = xErr;
    lastYErr = yErr;
    lastHErr = hErr;

    // -----------------------------
    // Outputs
    // -----------------------------
    double xOut = kP_xy * xErr + kD_xy * dxErr;
    double yOut = kP_xy * yErr + kD_xy * dyErr;
    double hOut = kP_h  * hErr + kD_h  * dhErr;

    // Dynamic translation limit
    const int dynMaxDrive =
      (int)std::round(std::min<double>(
        maxDrive,
        std::max<double>(minDriveNear, dist * kSlow)
      ));

    xOut = clampd(xOut, -dynMaxDrive, dynMaxDrive);
    yOut = clampd(yOut, -dynMaxDrive, dynMaxDrive);
    hOut = clampd(hOut, -(double)maxTurn, (double)maxTurn);

    // Minimum translation power (beats friction)
    if (std::fabs(xErr) > posTolIn && std::fabs(xOut) < MIN_XY) xOut = sgn(xErr) * MIN_XY;
    if (std::fabs(yErr) > posTolIn && std::fabs(yOut) < MIN_XY) yOut = sgn(yErr) * MIN_XY;

    // Arc reduction: when cross-track is big, don't let heading “steer” you into an arc
    if (std::fabs(xErr) > ARC_CROSS_IN) {
      hOut = clampd(hOut, -ARC_H_MAX, ARC_H_MAX);
    }

    // Minimum turn (only if you actually want heading correction)
    if (std::fabs(hErr) > headTolRad && std::fabs(hOut) < MIN_T) hOut = sgn(hErr) * MIN_T;

    // -----------------------------
    // Mix (X-drive)
    // -----------------------------
    double fl = yOut + xOut + hOut;
    double fr = yOut - xOut - hOut;
    double bl = yOut - xOut + hOut;
    double br = yOut + xOut - hOut;

    // Normalize properly (keep <= 127)
    double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br)});
    if (maxMag > 127.0) {
      const double sc = 127.0 / maxMag;
      fl *= sc; fr *= sc; bl *= sc; br *= sc;
    }

    setDrivePower((int)fl, (int)fr, (int)bl, (int)br);

    // -----------------------------
    // Debug (Brain LCD + optional USB)
    // -----------------------------
    if (++lcdCounter >= 5) { // every ~100ms
      lcdCounter = 0;
      pros::lcd::print(0, "T(%.1f,%.1f) d:%.2f", targetX, targetY, dist);
      pros::lcd::print(1, "eX:%.2f eY:%.2f", xErr, yErr);
      pros::lcd::print(2, "h:%.1f e:%.2f", ch * 180.0 / M_PI, hErr * 180.0 / M_PI);
      pros::lcd::print(3, "y:%.0f x:%.0f r:%.0f", yOut, xOut, hOut);
      pros::lcd::print(4, "dyn:%d set:%d", dynMaxDrive, settleCount);
      pros::lcd::print(5, "t:%dms", (int)(now - start));
      // printf("[TP] x=%.2f y=%.2f h=%.1f  eX=%.2f eY=%.2f d=%.2f  outY=%.1f outX=%.1f outH=%.1f\n",
      //        cx, cy, ch*180/M_PI, xErr, yErr, dist, yOut, xOut, hOut);
    }

    pros::delay(loopMs);
  }

  setDrivePower(0, 0, 0, 0);
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

