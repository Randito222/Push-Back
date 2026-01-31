#include "AutoPathAsync.hpp"
#include "PathPlanner.hpp"
#include "PathFollower.hpp"
#include "main.h"
#include <string>
#include <cstring>

// Same helpers as AutoPath.cpp (kept local to avoid extra shared headers)
static bool looksLikeAssetBuffer(const char* s) {
  if (!s) return false;
  for (int i = 0; i < 256 && s[i]; i++) {
    if (s[i] == '\n') return true;
    if (s[i] == ',')  return true;
    if (s[i] == '#')  return true;
  }
  return false;
}

static const char* resolveSource(const char* filenameOrAsset, std::string& tmp) {
  if (!filenameOrAsset) return filenameOrAsset;
  if (looksLikeAssetBuffer(filenameOrAsset)) return filenameOrAsset;

  std::string f(filenameOrAsset);
  if (f.find('/') != std::string::npos) {
    tmp = f;
    return tmp.c_str();
  }

  tmp = "/usd/" + f;
  return tmp.c_str();
}

static FollowConfig defaults() {
  FollowConfig cfg;
  cfg.lookaheadIn = 7.0;     // tighter path lock
  cfg.maxSpeed    = 90;      // reduce overshoot
  cfg.slewRateV   = 5;       // smoother corrections

  cfg.kP_xy = 9.0;
  cfg.kI_xy = 0.0;
  cfg.kD_xy = 30.0;

  cfg.kP_turn = 2.0;
  cfg.kI_turn = 0.0;
  cfg.kD_turn = 10.0;

  cfg.endDistIn = 0.6;
  cfg.usePointSpeed = true;
  cfg.anchorToRobotPose = true;
  return cfg;
}

void followPathAsync(const char* filenameOrAsset, HeadingMode headingMode, double finalHeadingDeg) {
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
  PathFollower::followPathAsync(loaded, headingMode, finalHeadingDeg, cfg);
}

bool isPathFollowing() {
  return PathFollower::isFollowing();
}

void waitPathDone(int timeout_ms) {
  PathFollower::waitUntilDone(timeout_ms);
}

void cancelPath() {
  PathFollower::cancel();
}
