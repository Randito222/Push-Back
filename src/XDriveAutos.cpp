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
  // Forward approach (robot forward)
  const double kP_y = 7.0;
  const double kD_y = 18.0;

  // X line lock BASE gains (will be scaled by mode multipliers)
  const double kP_x = 32.0;
  const double kD_x = 140.0;
  const double kI_x = 0.45;

  // Heading (deg)
  const double kP_h = 2.4;
  const double kD_h = 9.0;

  // =============================
  // TOLERANCES + SETTLE
  // =============================
  const double posTolIn    = 1.0;
  const double headTolDeg  = 2.0;
  const int    settleReq   = 10;

  // =============================
  // MIN OUTPUTS
  // =============================
  const double MIN_XY = 10.0;
  const double MIN_T  = 7.0;

  // =============================
  // I-ZONE / ANTI-WINDUP for X lock
  // =============================
  const double xIZoneIn = 6.0;
  const double xIMax    = 80.0;

  // =============================
  // DYNAMIC LIMITS
  // =============================
  const double kSlow = 6.0;
  const int minDriveNear = 12;

  // =============================
  // ARC REDUCTION
  // =============================
  const double ARC_CROSS_IN = 0.75;
  const double ARC_H_MAX    = 10.0;

  // =============================
  // SNAP + HOLD BANDS (this fixes oscillation)
  // =============================
  const double X_SNAP_IN   = 1.0;   // enter SNAP when |x| > 1"
  const double X_HOLD_IN   = 0.35;  // return to HOLD when |x| < 0.35"
  const double X_DEADBAND  = 0.30;  // inside this, treat x as 0 (no chatter)

  // Mode gain multipliers:
  const double X_SNAP_MUL  = 1.25;  // strong correction in SNAP
  const double X_HOLD_MUL  = 0.45;  // gentle hold near line (kills hunting)

  // Kick only when entering SNAP
  const int    X_KICK_LOOPS = 3;    // 60ms
  const double X_KICK_PWR   = 10.0; // small kick

  // Near-finish priority (lets Y close)
  const double Y_CLOSE_IN   = 4.0;
  const double X_NEAR_SCALE = 0.60;

  // =============================
  // STATE
  // =============================
  double lastYErr = 0.0;
  double lastXLineRobot = 0.0;
  double lastHErr = 0.0;

  double xI = 0.0;
  int settleCount = 0;
  int lcdCounter = 0;

  int xKick = 0;

  enum class XMode { HOLD, SNAP };
  XMode xMode = XMode::HOLD;

  while (true) {
    const uint32_t now = pros::millis();
    if ((int)(now - start) > timeoutMs) break;

    // Pose
    const double cx = odomX;
    const double cy = odomY;
    const double ch = wrapPi(odomTheta);

    // Field error
    const double fxErr = targetX - cx;
    const double fyErr = targetY - cy;
    const double dist  = std::hypot(fxErr, fyErr);

    // Field -> Robot (inverse)
    const double c = std::cos(ch);
    const double s = std::sin(ch);

    const double yErr = -fxErr * s + fyErr * c;

    // X line error (field X)
    const double xLineErrRaw = targetX - cx;

    // -----------------------------
    // MODE STATE MACHINE (SNAP early, HOLD to stop oscillation)
    // -----------------------------
    if (xMode == XMode::HOLD) {
      if (std::fabs(xLineErrRaw) > X_SNAP_IN) {
        xMode = XMode::SNAP;
        xKick = X_KICK_LOOPS;   // kick only ON ENTER
      }
    } else { // SNAP
      if (std::fabs(xLineErrRaw) < X_HOLD_IN) {
        xMode = XMode::HOLD;
        xKick = 0;              // no kick in HOLD
      }
    }

    // Deadband to kill chatter when basically on the line
    double xLineErr = xLineErrRaw;
    if (std::fabs(xLineErr) < X_DEADBAND) xLineErr = 0.0;

    // Project field X correction into robot strafe axis
    const double xLineRobot = xLineErr * std::cos(ch);

    // Heading error (deg)
    const double curDeg = IMU.get_rotation();
    const double hErr   = wrapDeg(targetHeadingDeg - curDeg);

    // Exit settle
    const bool posOK  = (dist < posTolIn);
    const bool headOK = (std::fabs(hErr) < headTolDeg);

    if (posOK && headOK) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleReq) break;

    // Derivatives
    const double dyErr  = (yErr - lastYErr);
    const double dxLine = (xLineRobot - lastXLineRobot);
    const double dhErr  = (hErr - lastHErr);

    lastYErr = yErr;
    lastXLineRobot = xLineRobot;
    lastHErr = hErr;

    // Integral (only near line, and don't integrate deadband=0)
    if (std::fabs(xLineErrRaw) < xIZoneIn && xLineErr != 0.0) {
      xI += xLineRobot;
      xI = clampd(xI, -xIMax, xIMax);
    } else if (std::fabs(xLineErrRaw) >= xIZoneIn) {
      xI = 0.0;
    }

    // -----------------------------
    // Outputs
    // -----------------------------
    double yOut = kP_y * yErr + kD_y * dyErr;

    // X mode multiplier
    const double xMul = (xMode == XMode::SNAP) ? X_SNAP_MUL : X_HOLD_MUL;

    // Strafe output (snap or hold)
    double xOut = xMul * (kP_x * xLineRobot + kD_x * dxLine + kI_x * xI);

    // Kick only during SNAP entry window
    if (xMode == XMode::SNAP && xKick > 0) {
      xOut += sgn(xLineRobot) * X_KICK_PWR;
      xKick--;
    }

    double hOut = kP_h * hErr + kD_h * dhErr;

    // Dynamic limits
    const int dynMaxDrive =
      (int)std::round(std::min<double>(
        maxDrive,
        std::max<double>(minDriveNear, dist * kSlow)
      ));

    // Let Y finish near the end
    if (std::fabs(yErr) < Y_CLOSE_IN) xOut *= X_NEAR_SCALE;

    xOut = clampd(xOut, -dynMaxDrive, dynMaxDrive);
    yOut = clampd(yOut, -dynMaxDrive, dynMaxDrive);
    hOut = clampd(hOut, -(double)maxTurn, (double)maxTurn);

    // Minimum powers (only if error exists)
    if (std::fabs(xLineErrRaw) > posTolIn && std::fabs(xOut) < MIN_XY) xOut = sgn(xLineRobot) * MIN_XY;
    if (std::fabs(yErr)       > posTolIn && std::fabs(yOut) < MIN_XY) yOut = sgn(yErr) * MIN_XY;

    if (std::fabs(xLineErrRaw) > ARC_CROSS_IN) hOut = clampd(hOut, -ARC_H_MAX, ARC_H_MAX);
    if (std::fabs(hErr) > headTolDeg && std::fabs(hOut) < MIN_T) hOut = sgn(hErr) * MIN_T;

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
    // if (++lcdCounter >= 5) {
    //   lcdCounter = 0;
    //   pros::lcd::print(0, "d:%.2f xRaw:%.2f y:%.2f", dist, xLineErrRaw, yErr);
    //   pros::lcd::print(1, "mode:%s kick:%d", (xMode==XMode::SNAP)?"SNAP":"HOLD", xKick);
    //   pros::lcd::print(2, "xOut:%.0f yOut:%.0f hOut:%.0f", xOut, yOut, hOut);
    //   pros::lcd::print(3, "h:%.1f eH:%.2f", curDeg, hErr);
    //   pros::lcd::print(4, "dyn:%d set:%d", dynMaxDrive, settleCount);
    //   pros::lcd::print(5, "t:%dms", (int)(now - start));
    // }

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