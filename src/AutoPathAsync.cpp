#include "AutoPathAsync.hpp"
#include "PathPlanner.hpp"
#include "PathFollower.hpp"
#include "main.h"

static pros::Task* pathTask = nullptr;
static volatile bool running = false;
static volatile bool cancelReq = false;

static LoadedPath loaded;
static HeadingMode mode;
static double finalHead;

static FollowConfig cfgDefault() {
  FollowConfig cfg;
  cfg.lookaheadIn = 10;
  cfg.maxSpeed = 110;
  cfg.timeout_ms = 6000;
  cfg.slewRateV = 8;
  cfg.kP_xy = 8; cfg.kI_xy = 0; cfg.kD_xy = 25;
  cfg.kP_turn = 2.5; cfg.kI_turn = 0; cfg.kD_turn = 14;
  cfg.endDistIn = 0.8;
  cfg.endHeadDeg = 2.0;
  cfg.usePointSpeed = true;
  return cfg;
}

// Optional: to support cancel, your PathFollower loop should check cancelReq.
// If you want, I’ll show the 2-line change to PathFollower to stop early.

static void taskFn(void*) {
  running = true;
  cancelReq = false;

  FollowConfig cfg = cfgDefault();
  // NOTE: Add a cancel check inside PathFollower if you want true cancel support
  PathFollower::followPath(loaded, mode, finalHead, cfg);

  running = false;
}

void followPathAsync(const char* filename, HeadingMode headingMode, double finalHeadingDeg) {
  if (running) return; // or cancel+restart if you prefer

  loaded = PathPlanner::loadFromFile(filename);
  if (!loaded.ok) {
    pros::lcd::print(0, "PATH LOAD FAIL");
    pros::lcd::print(1, "%s", loaded.err.c_str());
    return;
  }

  mode = headingMode;
  finalHead = finalHeadingDeg;

  if (pathTask) { delete pathTask; pathTask = nullptr; }
  pathTask = new pros::Task(taskFn, nullptr, "PathFollow");
}

bool isPathFollowing() { return running; }

void waitPathDone(int timeout_ms) {
  int start = pros::millis();
  while (running && (pros::millis() - start < timeout_ms)) {
    pros::delay(10);
  }
}

void cancelPath() {
  cancelReq = true;
  // If you add cancel checks inside PathFollower, it will stop quickly.
}
