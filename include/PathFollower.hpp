#pragma once
#include "PathPlanner.hpp"
#include "XDrive_PID.hpp"   // for HeadingMode enum
#include <cstdint>

struct FollowConfig {
  double lookaheadIn = 10.0;   // 8..14 common for holonomic
  int    maxSpeed    = 110;    // -127..127
  int    timeout_ms  = 6000;
  double slewRateV   = 8;      // per 10ms loop

  // Translation PID (inches of robot-frame error)
  double kP_xy = 8.0;
  double kI_xy = 0.0;
  double kD_xy = 25.0;

  // Turn PID (degrees)
  double kP_turn = 2.5;
  double kI_turn = 0.0;
  double kD_turn = 14.0;

  // End condition
  double endDistIn  = 0.8;     // inches to final point
  double endHeadDeg = 2.0;     // degrees if heading controlled

  // Path speed column behavior
  bool usePointSpeed = true;

  // Unit conversion + anchoring
  bool   assumeMillimeters = true; // your file numbers like 162.291 strongly suggest mm
  bool   anchorToRobotPose = true; // normalize so first point starts at current odom pose
};

namespace PathFollower {
  // Blocking follow
  void followPath(
      LoadedPath path,                 // passed by value so we can modify/normalize safely
      HeadingMode headingMode,
      double finalHeadingDeg,
      const FollowConfig& cfg
  );

  // Async control (optional)
  void followPathAsync(
      LoadedPath path,
      HeadingMode headingMode,
      double finalHeadingDeg,
      const FollowConfig& cfg
  );

  bool isFollowing();
  void cancel();
  void waitUntilDone(int timeout_ms = 6000);
}
