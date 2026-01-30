#include "PathFollower.hpp"
#include "OdomSet.hpp"
#include "XDrive_PID.hpp"
#include "subsystems.hpp"
#include "main.h"
#include <cmath>
#include <algorithm>

// If you already have these utilities globally, you can remove these and use yours.
static double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static double wrapDeg(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

// Motor aliases (match your subsystems.hpp names)
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

static void stopDriveLocal() {
  FL1.move(0); FL2.move(0);
  FR1.move(0); FR2.move(0);
  BL1.move(0); BL2.move(0);
  BR1.move(0); BR2.move(0);
}

static double MyslewLocal(double target, double current, double step) {
  double diff = target - current;
  if (std::fabs(diff) <= step) return target;
  return current + (diff > 0 ? step : -step);
}

struct PID {
  double kP=0,kI=0,kD=0;
  double i=0, prev=0;
  double iLimit=1000;
  double step(double e) {
    i += e;
    i = clampd(i, -iLimit, iLimit);
    double d = e - prev;
    prev = e;
    return kP*e + kI*i + kD*d;
  }
  void reset() { i=0; prev=0; }
};

// Closest-point search (incremental)
static size_t closestIndex(const std::vector<PathPoint>& pts, double x, double y, size_t startIdx) {
  size_t best = startIdx;
  double bestD2 = 1e18;
  for (size_t i = startIdx; i < pts.size(); i++) {
    double dx = pts[i].x - x;
    double dy = pts[i].y - y;
    double d2 = dx*dx + dy*dy;
    if (d2 < bestD2) { bestD2 = d2; best = i; }
  }
  return best;
}

static PathPoint lookaheadPoint(const std::vector<PathPoint>& pts, double x, double y, size_t fromIdx, double lookaheadIn) {
  PathPoint last = pts.back();
  for (size_t i = fromIdx; i < pts.size(); i++) {
    double dx = pts[i].x - x;
    double dy = pts[i].y - y;
    if (std::hypot(dx, dy) >= lookaheadIn) return pts[i];
  }
  return last;
}

static void normalizeJerry(LoadedPath& path, const FollowConfig& cfg) {
  // Convert mm->in if requested
  const double MM_TO_IN = 1.0 / 25.4;
  const double scale = cfg.assumeMillimeters ? MM_TO_IN : 1.0;

  // If anchoring, shift so first point is (0,0) and then add current odom pose
  const double x0 = path.pts.front().x;
  const double y0 = path.pts.front().y;

  for (auto &p : path.pts) {
    double localX = (p.x - x0) * scale;
    double localY = (p.y - y0) * scale;

    if (cfg.anchorToRobotPose) {
      p.x = odomX + localX;
      p.y = odomY + localY;
    } else {
      // Just unit-convert
      p.x = p.x * scale;
      p.y = p.y * scale;
    }
  }
}

// --------------------
// Core follower loop
// --------------------
static volatile bool g_running = false;
static volatile bool g_cancel  = false;

static void runFollower(
    LoadedPath path,
    HeadingMode headingMode,
    double finalHeadingDeg,
    const FollowConfig& cfg
) {
  if (!path.ok || path.pts.size() < 2) return;

  // Normalize/anchor path.jerryio export to your odom frame
  normalizeJerry(path, cfg);

  PIDTest xPID{cfg.kP_xy, cfg.kI_xy, cfg.kD_xy};
  PIDTest yPID{cfg.kP_xy, cfg.kI_xy, cfg.kD_xy};
  PIDTest tPID{cfg.kP_turn, cfg.kI_turn, cfg.kD_turn};
  xPID.reset(); yPID.reset(); tPID.reset();

  const double holdHeadingDeg = odomTheta * 180.0 / M_PI;

  double fl=0, fr=0, bl=0, br=0;
  int start = pros::millis();
  int settled = 0;
  size_t idx = 0;

  while (pros::millis() - start < cfg.timeout_ms) {
    if (g_cancel) break;

    idx = closestIndex(path.pts, odomX, odomY, idx);
    PathPoint la = lookaheadPoint(path.pts, odomX, odomY, idx, cfg.lookaheadIn);

    // Error to lookahead in FIELD frame
    double xErr = la.x - odomX;
    double yErr = la.y - odomY;

    // End distance to final point
    double endDx = path.pts.back().x - odomX;
    double endDy = path.pts.back().y - odomY;
    double distToEnd = std::hypot(endDx, endDy);

    // Heading target
    double headingDeg = odomTheta * 180.0 / M_PI;
    double headingTarget = finalHeadingDeg;

    if (headingMode == HeadingMode::HOLD) {
      headingTarget = holdHeadingDeg;
    } else if (headingMode == HeadingMode::FACE_TARGET) {
      // 0° = +Y in your convention => atan2(x, y)
      headingTarget = std::atan2(xErr, yErr) * 180.0 / M_PI;
    } else {
      headingTarget = finalHeadingDeg;
    }

    double tErr = wrapDeg(headingTarget - headingDeg);

    // End settle
    bool headOk = (headingMode == HeadingMode::HOLD) ? true : (std::fabs(tErr) < cfg.endHeadDeg);
    if (distToEnd < cfg.endDistIn && headOk) {
      settled += 10;
      if (settled > 200) break;
    } else settled = 0;

    // FIELD -> ROBOT transform (same as your DriveToPoint_OdomPID)
    double sinH = std::sin(odomTheta);
    double cosH = std::cos(odomTheta);
    double robotX =  xErr * cosH + yErr * sinH;   // strafe
    double robotY = -xErr * sinH + yErr * cosH;   // forward

    // Speed scaling
    double vScale = 1.0;
    if (cfg.usePointSpeed && std::isfinite(la.speed)) {
      // Your file speed looks like 0..600 (editor units). We clamp and map it.
      // If you prefer, you can set usePointSpeed=false.
      vScale = clampd(la.speed / 600.0, 0.25, 1.0);
    }

    // Slow down near end
    double endScale = clampd(distToEnd / 18.0, 0.25, 1.0);
    double maxSp = cfg.maxSpeed * vScale * endScale;

    double xOut = clampd(xPID.step(robotX), -maxSp, maxSp);
    double yOut = clampd(yPID.step(robotY), -maxSp, maxSp);

    double moveMag = std::hypot(robotX, robotY);
    double turnScale = clampd(1.0 - (moveMag / 24.0), 0.25, 1.0);
    double tOut = clampd(tPID.step(tErr) * turnScale, -maxSp, maxSp);

    // X-drive mix
    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    // Normalize
    double maxMag = std::max({std::fabs(tFL), std::fabs(tFR), std::fabs(tBL), std::fabs(tBR)});
    if (maxMag > maxSp) {
      double s = maxSp / maxMag;
      tFL *= s; tFR *= s; tBL *= s; tBR *= s;
    }

    // Slew + apply
    fl = MyslewLocal(tFL, fl, cfg.slewRateV);
    fr = MyslewLocal(tFR, fr, cfg.slewRateV);
    bl = MyslewLocal(tBL, bl, cfg.slewRateV);
    br = MyslewLocal(tBR, br, cfg.slewRateV);

    FL1.move((int)fl); FL2.move((int)fl);
    FR1.move((int)fr); FR2.move((int)fr);
    BL1.move((int)bl); BL2.move((int)bl);
    BR1.move((int)br); BR2.move((int)br);

    // Debug
    pros::lcd::print(2, "idx:%d end:%.1f la(%.1f,%.1f)", (int)idx, distToEnd, la.x, la.y);
    pros::lcd::print(3, "errF x%.1f y%.1f | errR rx%.1f ry%.1f", xErr, yErr, robotX, robotY);
    pros::lcd::print(4, "head %.1f targ %.1f tErr %.1f", headingDeg, headingTarget, tErr);

    pros::delay(10);
  }

  stopDriveLocal();
}

void PathFollower::followPath(
    LoadedPath path,
    HeadingMode headingMode,
    double finalHeadingDeg,
    const FollowConfig& cfg
) {
  g_cancel = false;
  runFollower(path, headingMode, finalHeadingDeg, cfg);
}

// -------- async --------
static pros::Task* g_task = nullptr;
static LoadedPath  g_path;
static HeadingMode g_mode;
static double      g_finalHead;
static FollowConfig g_cfg;

static void taskFn(void*) {
  g_running = true;
  g_cancel = false;
  runFollower(g_path, g_mode, g_finalHead, g_cfg);
  g_running = false;
}

void PathFollower::followPathAsync(
    LoadedPath path,
    HeadingMode headingMode,
    double finalHeadingDeg,
    const FollowConfig& cfg
) {
  if (g_running) return; // or cancel+restart if you want

  g_path = path;
  g_mode = headingMode;
  g_finalHead = finalHeadingDeg;
  g_cfg = cfg;

  if (g_task) { delete g_task; g_task = nullptr; }
  g_task = new pros::Task(taskFn, nullptr, "PathFollowAsync");
}

bool PathFollower::isFollowing() { return g_running; }

void PathFollower::cancel() { g_cancel = true; }

void PathFollower::waitUntilDone(int timeout_ms) {
  int start = pros::millis();
  while (g_running && (pros::millis() - start < timeout_ms)) pros::delay(10);
}
