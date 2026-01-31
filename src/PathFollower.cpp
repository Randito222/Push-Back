#include "PathFollower.hpp"
#include "OdomSet.hpp"
#include "XDrive_PID.hpp"
#include "subsystems.hpp"
#include "main.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================
// Small helpers
// =============================
static double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static double wrapDeg(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

static double radToDeg(double r) { return r * 180.0 / M_PI; }

static const char* headingModeStr(HeadingMode m) {
  switch (m) {
    case HeadingMode::ABSOLUTE:    return "ABS";
    case HeadingMode::HOLD:        return "HOLD";
    case HeadingMode::FACE_TARGET: return "FACE";
    default:                       return "?";
  }
}

// Print at most every N ms (prevents LCD spam + keeps loop smooth)
static bool dbgTick(int periodMs = 120) {
  static int last = 0;
  int now = pros::millis();
  if (now - last < periodMs) return false;
  last = now;
  return true;
}

// =============================
// Motor aliases (match subsystems.hpp)
// =============================
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

// =============================
// Closest-point search (incremental)
// =============================
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

static PathPoint lookaheadPoint(const std::vector<PathPoint>& pts,
                                double x, double y,
                                size_t fromIdx,
                                double lookaheadIn) {
  PathPoint last = pts.back();
  for (size_t i = fromIdx; i < pts.size(); i++) {
    double dx = pts[i].x - x;
    double dy = pts[i].y - y;
    if (std::hypot(dx, dy) >= lookaheadIn) return pts[i];
  }
  return last;
}

// =============================
// Normalize Jerry path:
// If anchorToRobotPose = true, treat first point as (0,0) and
// translate the whole path to current odom pose.
// Coordinate convention in this project:
//   odomX = forward axis
//   odomY = right axis
// =============================
static void normalizeJerry(LoadedPath& path, const FollowConfig& cfg) {
  if (path.pts.size() < 2) return;

  const double x0 = path.pts.front().x;
  const double y0 = path.pts.front().y;

  for (auto& p : path.pts) {
    const double localX = p.x - x0;
    const double localY = p.y - y0;

    if (cfg.anchorToRobotPose) {
      p.x = odomX + localX;
      p.y = odomY + localY;
    }
  }
}

// =============================
// Core follower loop control
// =============================
static volatile bool g_running = false;
static volatile bool g_cancel  = false;

// Debug paging / latch
static int g_dbgPage = 0;          // 0 or 1
static int g_badLatchUntil = 0;    // timestamp until which we force BAD page

static void runFollower(
    LoadedPath path,
    HeadingMode headingMode,
    double finalHeadingDeg,
    const FollowConfig& cfg
) {
  if (!path.ok || path.pts.size() < 2) return;

  normalizeJerry(path, cfg);

  PIDTest fPID{cfg.kP_xy,   cfg.kI_xy,   cfg.kD_xy};    // forward axis PID
  PIDTest rPID{cfg.kP_xy,   cfg.kI_xy,   cfg.kD_xy};    // right axis PID
  PIDTest tPID{cfg.kP_turn, cfg.kI_turn, cfg.kD_turn};  // turn PID
  fPID.reset(); rPID.reset(); tPID.reset();

  const double holdHeadingDeg = radToDeg(odomTheta);

  double fl=0, fr=0, bl=0, br=0;
  int start = pros::millis();
  int settled = 0;
  size_t idx = 0;

  while (pros::millis() - start < cfg.timeout_ms) {
    if (g_cancel) break;

    // Current pose in VEX GPS axes (your project mapping)
    const double fNow = odomX;   // forward axis
    const double rNow = odomY;   // right axis

    // Find closest and lookahead
    idx = closestIndex(path.pts, fNow, rNow, idx);
    PathPoint la = lookaheadPoint(path.pts, fNow, rNow, idx, cfg.lookaheadIn);

    // Closest point distance (diagnostic)
    const double cdx = path.pts[idx].x - fNow;
    const double cdy = path.pts[idx].y - rNow;
    const double closestDist = std::hypot(cdx, cdy);

    // Detect if lookahead fell back to last point
    const bool laIsLast = (la.x == path.pts.back().x && la.y == path.pts.back().y);

    // Error to lookahead in FIELD frame (VEX GPS axes)
    const double fErr = la.x - fNow; // forward error
    const double rErr = la.y - rNow; // right error

    // End distance to final point
    const double endDf = path.pts.back().x - fNow;
    const double endDr = path.pts.back().y - rNow;
    const double distToEnd = std::hypot(endDf, endDr);

    // Heading target (0 deg points along +odomX, positive towards +odomY)
    const double headingDeg = radToDeg(odomTheta);
    double headingTarget = finalHeadingDeg;

    if (headingMode == HeadingMode::HOLD) {
      headingTarget = holdHeadingDeg;
    } else if (headingMode == HeadingMode::FACE_TARGET) {
      // 0° = +forward => atan2(right, forward)
      headingTarget = std::atan2(rErr, fErr) * 180.0 / M_PI;
    } else {
      headingTarget = finalHeadingDeg;
    }

    const double tErr = wrapDeg(headingTarget - headingDeg);

    // End settle
    const bool headOk = (headingMode == HeadingMode::HOLD) ? true : (std::fabs(tErr) < cfg.endHeadDeg);
    if (distToEnd < cfg.endDistIn && headOk) {
      settled += 10;
      if (settled > 200) break;
    } else {
      settled = 0;
    }

    // FIELD -> ROBOT transform (field axes: forward=fErr, right=rErr)
    const double sinH = std::sin(odomTheta);
    const double cosH = std::cos(odomTheta);

    // Robot frame:
    //   robotForward =  fErr*cos + rErr*sin
    //   robotRight   = -fErr*sin + rErr*cos
    const double robotForward =  fErr * cosH + rErr * sinH;
    const double robotRight   = -fErr * sinH + rErr * cosH;

    // Speed limiting
    double maxSp = cfg.maxSpeed;
    const bool hasPtSpeed = (cfg.usePointSpeed && std::isfinite(la.speed));
    if (hasPtSpeed) {
      double ptMax = clampd(std::fabs(la.speed), 10.0, cfg.maxSpeed);
      maxSp = std::min(maxSp, ptMax);
    }

    const double endScale = clampd(distToEnd / 18.0, 0.25, 1.0);
    maxSp *= endScale;

    // RAW PID outputs (before clamp)
    const double fRaw = fPID.step(robotForward);
    const double rRaw = rPID.step(robotRight);

    // Turn scaling while moving
    const double moveMag = std::hypot(robotForward, robotRight);
    const double turnScale = clampd(1.0 - (moveMag / 24.0), 0.25, 1.0);
    const double tRaw = tPID.step(tErr);

    // Clamped outputs
    double fOut = clampd(fRaw, -maxSp, maxSp);
    double rOut = clampd(rRaw, -maxSp, maxSp);
    double tOut = clampd(tRaw * turnScale, -maxSp, maxSp);

    // Suppress turning briefly at start to prevent initial spin
    const int elapsed = pros::millis() - start;
    if (elapsed < 300) tOut = 0;

    // X-drive mix:
    // yOut = forward, xOut = right/strafe
    const double yOut = fOut;
    const double xOut = rOut;

    double tFL = yOut + xOut + tOut;
    double tFR = yOut - xOut - tOut;
    double tBL = yOut - xOut + tOut;
    double tBR = yOut + xOut - tOut;

    // Normalize to maxSp
    double m = std::max({std::fabs(tFL), std::fabs(tFR), std::fabs(tBL), std::fabs(tBR)});
    if (m > maxSp && m > 1e-6) {
      double s = maxSp / m;
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

    // Optional: toggle debug pages with LCD buttons
    if (pros::lcd::read_buttons() & LCD_BTN_LEFT)  g_dbgPage = 0;
    if (pros::lcd::read_buttons() & LCD_BTN_RIGHT) g_dbgPage = 1;

    // ------------------------
    // Unified Debug (1 tick, 2 pages, bad-state latch)
    // ------------------------
    bool bad = false;
    if (!std::isfinite(odomTheta) || !std::isfinite(fNow) || !std::isfinite(rNow)) bad = true;
    if (!std::isfinite(fErr) || !std::isfinite(rErr)) bad = true;
    if (std::hypot(fErr, rErr) > 144.0) bad = true;      // >12ft error
    if (closestDist > 48.0) bad = true;                  // far from path = wrong anchor/odom

    if (bad) g_badLatchUntil = pros::millis() + 1500;    // show BAD for 1.5s
    const bool showBad = (pros::millis() < g_badLatchUntil);

    if (dbgTick(120)) {
      const double ptMaxRaw = hasPtSpeed ? std::fabs(la.speed) : NAN;

      const char* reason = "NORMAL";
      if (g_cancel) reason = "CANCEL";
      else if (elapsed < 300) reason = "START_NO_TURN";
      else if (distToEnd < cfg.endDistIn && headOk) reason = "END_SETTLE";
      else if (distToEnd < cfg.endDistIn) reason = "END_ZONE";
      else if (laIsLast) reason = "LA_LAST";

      int page = g_dbgPage;
      if (showBad) page = 99;

      if (page == 99) {
        pros::lcd::print(0, "!!! BAD STATE !!! (%s)", reason);
        pros::lcd::print(1, "Pose f%.1f r%.1f h%.1f", fNow, rNow, headingDeg);
        pros::lcd::print(2, "ErrF f%.1f r%.1f end%.1f", fErr, rErr, distToEnd);
        pros::lcd::print(3, "idx %d cd%.1f LA(%.1f,%.1f)", (int)idx, closestDist, la.x, la.y);
        pros::lcd::print(4, "Head %.1f->%.1f e%.1f", headingDeg, headingTarget, tErr);
        pros::lcd::print(5, "maxSp%.0f endS%.2f turnS%.2f", maxSp, endScale, turnScale);
        pros::lcd::print(6, "raw f%.0f r%.0f t%.0f", fRaw, rRaw, tRaw);
        pros::lcd::print(7, "mot FL%.0f FR%.0f BL%.0f BR%.0f", fl, fr, bl, br);
      }
      else if (page == 0) {
        pros::lcd::print(0, "%s idx:%d/%d t:%dms %s",
                         headingModeStr(headingMode),
                         (int)idx, (int)path.pts.size(),
                         elapsed, reason);

        pros::lcd::print(1, "Pose f%.1f r%.1f h%.1f", fNow, rNow, headingDeg);

        pros::lcd::print(2, "LA f%.1f r%.1f end%.2f cd%.2f",
                         la.x, la.y, distToEnd, closestDist);

        pros::lcd::print(3, "ErrF f%.1f r%.1f | ErrR f%.1f r%.1f",
                         fErr, rErr, robotForward, robotRight);

        pros::lcd::print(4, "Head %.1f->%.1f e%.1f", headingDeg, headingTarget, tErr);

        pros::lcd::print(5, "maxSp%.0f pt%.0f endS%.2f",
                         maxSp,
                         std::isfinite(ptMaxRaw) ? ptMaxRaw : -1.0,
                         endScale);

        pros::lcd::print(6, "Out f%.0f r%.0f t%.0f mv%.1f",
                         fOut, rOut, tOut, moveMag);

        pros::lcd::print(7, "M FL%.0f FR%.0f BL%.0f BR%.0f",
                         fl, fr, bl, br);
      }
      else {
        pros::lcd::print(0, "DEEP %s t:%dms", headingModeStr(headingMode), elapsed);
        pros::lcd::print(1, "hold%.1f final%.1f", holdHeadingDeg, finalHeadingDeg);
        pros::lcd::print(2, "ptMax%.0f use:%d", std::isfinite(ptMaxRaw) ? ptMaxRaw : -1.0, (int)cfg.usePointSpeed);
        pros::lcd::print(3, "turnS%.2f mv%.1f", turnScale, moveMag);
        pros::lcd::print(4, "raw f%.0f r%.0f t%.0f", fRaw, rRaw, tRaw);
        pros::lcd::print(5, "out f%.0f r%.0f t%.0f", fOut, rOut, tOut);
        pros::lcd::print(6, "settle%d headOk%d", settled, (int)headOk);
        pros::lcd::print(7, "laLast%d idx%d cd%.1f", (int)laIsLast, (int)idx, closestDist);
      }
    }

    pros::delay(10);
  }

  stopDriveLocal();
}

// =============================
// Public API
// =============================
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
static LoadedPath   g_path;
static HeadingMode  g_mode;
static double       g_finalHead;
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
  if (g_running) return;

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
