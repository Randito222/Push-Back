#include "AutoPath.hpp"
#include "PathPlanner.hpp"
#include "PathFollower.hpp"
#include "main.h"
#include <string>

// Default follower settings (tune once)
static FollowConfig defaults() {
  FollowConfig cfg;
  cfg.lookaheadIn = 10.0;
  cfg.maxSpeed = 110;
  cfg.timeout_ms = 6000;
  cfg.slewRateV = 8;

  cfg.kP_xy = 8.0;
  cfg.kI_xy = 0.0;
  cfg.kD_xy = 25.0;

  cfg.kP_turn = 2.5;
  cfg.kI_turn = 0.0;
  cfg.kD_turn = 14.0;

  cfg.endDistIn = 0.8;
  cfg.endHeadDeg = 2.0;

  // Your export numbers match mm well; keep true unless you change editor units
  cfg.assumeMillimeters = true;

  // This is the magic that fixes “editor shows 0,0 but file starts at 162,-38”
  cfg.anchorToRobotPose = true;

  // If your speed column doesn’t feel right, set false
  cfg.usePointSpeed = true;

  return cfg;
}

// Helper: allow followPath("Right_PIckupBalls.txt") without typing /usd/
static std::string makeUsdPath(const char* filename) {
  std::string f(filename);
  if (f.rfind("/usd/", 0) == 0) return f; // already has prefix
  return "/usd/" + f;
}

void followPath(const char* filename) {
  followPath(filename, HeadingMode::FACE_TARGET, 0.0);
}

void followPath(const char* filename, HeadingMode mode, double finalHeadingDeg) {
  std::string path = makeUsdPath(filename);

  auto loaded = PathPlanner::loadJerry(path.c_str());
  if (!loaded.ok) {
    pros::lcd::print(0, "PATH LOAD FAIL");
    pros::lcd::print(1, "%s", loaded.err.c_str());
    pros::lcd::print(2, "%s", path.c_str());
    return;
  }

  FollowConfig cfg = defaults();
  PathFollower::followPath(loaded, mode, finalHeadingDeg, cfg);
}

void followPathAsync(const char* filename, HeadingMode mode, double finalHeadingDeg) {
  std::string path = makeUsdPath(filename);

  auto loaded = PathPlanner::loadJerry(path.c_str());
  if (!loaded.ok) {
    pros::lcd::print(0, "PATH LOAD FAIL");
    pros::lcd::print(1, "%s", loaded.err.c_str());
    pros::lcd::print(2, "%s", path.c_str());
    return;
  }

  FollowConfig cfg = defaults();
  PathFollower::followPathAsync(loaded, mode, finalHeadingDeg, cfg);
}

bool isPathFollowing() { return PathFollower::isFollowing(); }
void waitPathDone(int timeout_ms) { PathFollower::waitUntilDone(timeout_ms); }
void cancelPath() { PathFollower::cancel(); }
