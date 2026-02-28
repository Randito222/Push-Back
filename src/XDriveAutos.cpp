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

// =============================
// DRIVE TO POINT (ODOM X/Y TARGETS) — X-DRIVE
// Uses: odomX/odomY/odomTheta from pros::Task odomTask()
// IMPORTANT: This version assumes odomTheta is NEGATIVE (as in your odomTask)
// =============================
void driveToPoint_XDrive_PID(
    double targetX,
    double targetY,
    double targetHeadingDeg,
    int maxDrive,
    int maxTurn,
    int timeoutMs
) {
  const uint32_t start = pros::millis();
  const int loopMs = 20;

  // =============================
  // GAINS
  // =============================
  const double kP_y = 7.0;
  const double kD_y = 18.0;

  const double kP_x = 32.0;
  const double kD_x = 140.0;
  const double kI_x = 0.45;

  const double kP_h = 2.4;
  const double kD_h = 9.0;

  // =============================
  // TOLERANCES + SETTLE
  // =============================
  const double posTolIn    = 1.0;
  const double headTolDeg  = 2.0;
  const int    settleReq   = 10;

  // =============================
  // FRICTION MIN OUTPUTS
  // =============================
  const double MIN_XY = 10.0;
  const double MIN_T  = 7.0;

  // =============================
  // I-ZONE / ANTI-WINDUP for X lock
  // =============================
  const double xIZoneIn     = 6.0;
  const double xIMax        = 80.0;

  // =============================
  // DYNAMIC LIMITS (distance-based)
  // =============================
  const double kSlow = 6.0;
  const int minDriveNear = 12;

  // =============================
  // ARC REDUCTION
  // =============================
  const double ARC_CROSS_IN = 0.75;
  const double ARC_H_MAX    = 10.0;

  // =============================
  // EARLY SNAP (threshold + boost + kick)
  // =============================
  const double X_SNAP_IN     = 1.0;
  const double X_SNAP_BOOST  = 1.12;
  const int    X_KICK_LOOPS  = 3;
  const double X_KICK_PWR    = 10.0;

  // =============================
  // ANTI-OSCILLATION ADDITIONS
  // =============================
  const double X_DEADBAND_IN = 0.35;  // 0.25..0.5: stops hunting near the line
  const double Y_CLOSE_IN    = 4.0;   // when |yErr| < this, reduce x authority
  const double X_NEAR_SCALE  = 0.60;  // 0.5..0.8

  // =============================
  // STATE
  // =============================
  double lastYErr = 0.0;
  double lastXLineErr = 0.0;
  double lastHErr = 0.0;

  double xI = 0.0;

  int settleCount = 0;
  int lcdCounter  = 0;

  int xKick = 0;

  // for "kick only on threshold crossing"
  bool wasOverSnap = false;

  while (true) {
    const uint32_t now = pros::millis();
    if ((int)(now - start) > timeoutMs) break;

    // Current pose
    const double cx = odomX;
    const double cy = odomY;
    const double ch = wrapPi(odomTheta);

    // Field error
    const double fxErr = targetX - cx;
    const double fyErr = targetY - cy;
    const double dist  = std::hypot(fxErr, fyErr);

    // Field -> Robot (inverse rotation)
    const double c = std::cos(ch);
    const double s = std::sin(ch);

    const double xErr =  fxErr * c + fyErr * s;
    const double yErr = -fxErr * s + fyErr * c;

    // X line lock (field X)
    const double xLineErrRaw = targetX - cx;

    // ✅ DEAD-BAND so it doesn't hunt around x=targetX
    double xLineErr = xLineErrRaw;
    if (std::fabs(xLineErr) < X_DEADBAND_IN) xLineErr = 0.0;

    const double xLineRobot = xLineErr * std::cos(ch);

    // Early snap trigger + kick ONLY on threshold crossing
    double xGainMul = 1.0;
    const bool overSnap = (std::fabs(xLineErrRaw) > X_SNAP_IN);

    if (overSnap) xGainMul = X_SNAP_BOOST;

    // kick only when we cross from <=1" to >1"
    if (overSnap && !wasOverSnap) {
      xKick = X_KICK_LOOPS;
    }
    wasOverSnap = overSnap;

    // Heading error (deg)
    const double curDeg = IMU.get_rotation();
    const double hErr   = wrapDeg(targetHeadingDeg - curDeg);

    // Exit gates
    const bool posOK  = (dist < posTolIn);
    const bool headOK = (std::fabs(hErr) < headTolDeg);

    if (posOK && headOK) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleReq) break;

    // Derivatives
    const double dyErr  = (yErr - lastYErr);
    const double dxLine = (xLineRobot - lastXLineErr);
    const double dhErr  = (hErr - lastHErr);

    lastYErr     = yErr;
    lastXLineErr = xLineRobot;
    lastHErr     = hErr;

    // X integral (I-zone) - use RAW error for I-zone check but integrate DB value
    if (std::fabs(xLineErrRaw) < xIZoneIn) {
      xI += xLineRobot;
      xI = clampd(xI, -xIMax, xIMax);
    } else {
      xI = 0.0;
    }

    // Outputs
    double yOut = kP_y * yErr + kD_y * dyErr;

    double xOut = xGainMul * (kP_x * xLineRobot + kD_x * dxLine + kI_x * xI);

    // Kick pulse
    if (xKick > 0) {
      xOut += sgn(xLineRobot) * X_KICK_PWR;
      xKick--;
    }

    double hOut = kP_h * hErr + kD_h * dhErr;

    // Dynamic translation limit
    const int dynMaxDrive =
      (int)std::round(std::min<double>(
        maxDrive,
        std::max<double>(minDriveNear, dist * kSlow)
      ));

    // ✅ PRIORITY: let Y finish near the end (reduces “stuck yErr ~2”)
    if (std::fabs(yErr) < Y_CLOSE_IN) {
      xOut *= X_NEAR_SCALE;
    }

    xOut = clampd(xOut, -dynMaxDrive, dynMaxDrive);
    yOut = clampd(yOut, -dynMaxDrive, dynMaxDrive);
    hOut = clampd(hOut, -(double)maxTurn, (double)maxTurn);

    // Minimum translation power
    if (std::fabs(xLineErrRaw) > posTolIn && std::fabs(xOut) < MIN_XY)
      xOut = sgn(xOut == 0 ? xLineRobot : xOut) * MIN_XY;

    if (std::fabs(yErr) > posTolIn && std::fabs(yOut) < MIN_XY)
      yOut = sgn(yErr) * MIN_XY;

    // Arc reduction
    if (std::fabs(xLineErrRaw) > ARC_CROSS_IN) {
      hOut = clampd(hOut, -ARC_H_MAX, ARC_H_MAX);
    }

    // Minimum turn
    if (std::fabs(hErr) > headTolDeg && std::fabs(hOut) < MIN_T)
      hOut = sgn(hErr) * MIN_T;

    // Mix (X-drive)
    double fl = yOut + xOut + hOut;
    double fr = yOut - xOut - hOut;
    double bl = yOut - xOut + hOut;
    double br = yOut + xOut - hOut;

    double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br)});
    if (maxMag > 127.0) {
      const double sc = 127.0 / maxMag;
      fl *= sc; fr *= sc; bl *= sc; br *= sc;
    }

    setDrivePower((int)fl, (int)fr, (int)bl, (int)br);

    // Debug
    if (++lcdCounter >= 5) {
      lcdCounter = 0;
      pros::lcd::print(0, "T(%.1f,%.1f) d:%.2f", targetX, targetY, dist);
      pros::lcd::print(1, "xRaw:%.2f xDB:%.2f", xLineErrRaw, xLineErr);
      pros::lcd::print(2, "yErr:%.2f h:%.1f eH:%.2f", yErr, curDeg, hErr);
      pros::lcd::print(3, "y:%.0f x:%.0f r:%.0f", yOut, xOut, hOut);
      pros::lcd::print(4, "dyn:%d set:%d", dynMaxDrive, settleCount);
      pros::lcd::print(5, "kick:%d mul:%.2f", xKick, xGainMul);
    }

    pros::delay(loopMs);
  }

  setDrivePower(0, 0, 0, 0);
}

void turnToHeading_PID(double targetDeg, int maxTurn, int timeoutMs) {
  const uint32_t start = pros::millis();

  const double kP = 2.4;
  const double kD = 9.0;

  const double tol = 1.5;
  const double MIN_T = 7.0;

  const int loopMs = 10;
  const double dt = loopMs / 1000.0;

  double lastErr = 0;
  int settle = 0;

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    double cur = IMU.get_rotation();
    double err = wrapDeg(targetDeg - cur);

    if (fabs(err) < tol) settle++;
    else settle = 0;

    if (settle >= 6) break;

    double out = kP * err + kD * (err - lastErr) / dt;
    lastErr = err;

    out = clampd(out, -maxTurn, maxTurn);

    if (fabs(err) > tol && fabs(out) < MIN_T)
      out = sgn(err) * MIN_T;

    // CCW positive
    setDrivePower((int)out, (int)-out, (int)out, (int)-out);

    pros::delay(loopMs);
  }

  setDrivePower(0,0,0,0);
}