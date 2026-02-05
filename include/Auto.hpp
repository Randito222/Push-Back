#pragma once

#include "Drive.hpp"

// Uses Push-Back global odometry: odomX, odomY, odomTheta (from OdomSet.hpp).

// Drive to a field point (inches) while facing a target heading (degrees).
void driveToPoint(FieldXDrive& drive,
                  double tx,
                  double ty,
                  double targetHeadingDeg,
                  double maxMove = 0.9,
                  double maxRot  = 0.7,
                  int timeoutMs  = 2500);

// Turn in place to a target heading (degrees).
void turnToHeading(FieldXDrive& drive,
                   double targetHeadingDeg,
                   double maxRot = 0.7,
                   int timeoutMs = 1500);

// ------------------------------------------------------------
// Legacy PID interfaces (optional to keep during transition)
// If you're not using these anymore, you can delete them.
// ------------------------------------------------------------
namespace legacy {
  double clamp(double v, double lo, double hi);
  double Myslew(double target, double current, double maxDelta);
  void stopDrive();

  void DriveToPoint_PID(double targetX_in,
                        double targetY_in,
                        double targetHeading_deg,
                        int    maxVolt    = 12000,
                        int    timeout_ms = 3000,
                        double slewRateV  = 300);

  void DriveToPoint_OdomPID(double targetX_in,
                            double targetY_in,
                            double targetHeading_deg,
                            int    maxVolt    = 12000,
                            int    timeout_ms = 3000,
                            double slewRateV  = 300);
}  // namespace legacy
