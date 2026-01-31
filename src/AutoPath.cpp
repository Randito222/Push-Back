#include "AutoPath.hpp"
#include "PathPlanner.hpp"
#include "PathFollower.hpp"
#include "main.h"
#include <string>
#include <cstring>

// ----------------------------
// Detect ASSET() buffer vs filename
// ASSET() contents typically include commas/newlines very early.
// ----------------------------
static bool looksLikeAssetBuffer(const char* s) {
  if (!s) return false;
  for (int i = 0; i < 256 && s[i]; i++) {
    if (s[i] == '\n') return true;
    if (s[i] == ',')  return true;
    if (s[i] == '#')  return true;
  }
  return false;
}

// Helper: allow followPath("RightPickup.txt") without typing /usd/
// BUT do NOT touch ASSET buffers.
static const char* resolveSource(const char* filenameOrAsset, std::string& tmp) {
  if (!filenameOrAsset) return filenameOrAsset;

  // If this is an ASSET() buffer, pass through untouched.
  if (looksLikeAssetBuffer(filenameOrAsset)) return filenameOrAsset;

  std::string f(filenameOrAsset);

  // If already absolute-ish (contains '/'), don't prefix.
  if (f.find('/') != std::string::npos) {
    tmp = f;
    return tmp.c_str();
  }

  // Otherwise, assume SD card file in /usd/
  tmp = "/usd/" + f;
  return tmp.c_str();
}

// Default follower settings (tune once)
static FollowConfig defaults() {
  FollowConfig cfg;

  // Geometry / smoothness
  cfg.lookaheadIn = 10.0;   // 8..12 common
  cfg.slewRateV   = 7;      // 6..10 typical for holonomic

  // Limits
  cfg.maxSpeed   = 120;     // allow point speeds like 87 to work naturally
  cfg.timeout_ms = 7000;

  // Translation PID (robot-frame inches error)
  cfg.kP_xy = 7.5;
  cfg.kI_xy = 0.0;
  cfg.kD_xy = 24.0;

  // Turn PID (degrees)
  cfg.kP_turn = 2.2;
  cfg.kI_turn = 0.0;
  cfg.kD_turn = 12.0;

  // End condition
  cfg.endDistIn  = 1.0;
  cfg.endHeadDeg = 3.0;

  // Path speed column behavior
  cfg.usePointSpeed = true;

  // Anchoring: treat first path point as (0,0) placed at current odom pose
  cfg.anchorToRobotPose = true;

  return cfg;
}

void followPath(const char* filenameOrAsset) {
  followPath(filenameOrAsset, HeadingMode::FACE_TARGET, 0.0);
}

void followPath(const char* filenameOrAsset, HeadingMode mode, double finalHeadingDeg) {
  std::string tmp;
  const char* src = resolveSource(filenameOrAsset, tmp);

  auto loaded = PathPlanner::loadJerry(src);
  if (!loaded.ok) {
    pros::lcd::print(0, "PATH LOAD FAIL");
    pros::lcd::print(1, "%s", loaded.err.c_str());
    pros::lcd::print(2, "%s", looksLikeAssetBuffer(filenameOrAsset) ? "(ASSET buffer)" : src);
    pros::lcd::print(3, "pts=%d", (int)loaded.pts.size());
    return;
  }

  FollowConfig cfg = defaults();
  PathFollower::followPath(loaded, mode, finalHeadingDeg, cfg);
}
