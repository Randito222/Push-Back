#include "Auto.hpp"
#include "main.h"

#include "OdomSet.hpp"   // odomX, odomY, odomTheta
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
double deg2rad(double d) { return d * (M_PI / 180.0); }

double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

static inline double wrap2pi(double a){
  a = std::remainder(a, 2.0 * M_PI);
  if(a < 0) a += 2.0 * M_PI;
  return a;
}

// Simple PID (kept intentionally close to original feel)
struct PID2 {
  double kP{0}, kI{0}, kD{0};
  double i{0}, last{0};
  double iLimit{1e9};
  double step(double err) {
    i += err;
    i = std::clamp(i, -iLimit, iLimit);
    double d = err - last;
    last = err;
    return (kP * err) + (kI * i) + (kD * d);
  }
  void reset() { i = 0; last = 0; }
};

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }
}  // namespace

void driveToPoint(FieldXDrive& drive,
                  double tx,
                  double ty,
                  double targetHeadingDeg,
                  double maxMove,
                  double maxRot,
                  int timeoutMs) {

  PID2 distPID{0.2, 0.0, 0.12};
  PID2 headPID{2.2,  0.0, 4.5};

  distPID.reset();
  headPID.reset();

  const double targetH = wrapPi(deg2rad(targetHeadingDeg));
  const uint32_t start = pros::millis();

  const double posTolIn   = 1.0;
  const double headTolRad = deg2rad(3.0);

  const double ROT_DEADBAND_RAD  = deg2rad(1.5);
  const double DIST_TURN_FADE_IN = 10.0;
  const double TURN_180_FIX_RAD  = deg2rad(3.0);

  int settledMs = 0;

  uint32_t t = pros::millis();
  int printEvery = 0;

  while ((int)(pros::millis() - start) < timeoutMs) {
    const double dx = tx - odomX;
    const double dy = ty - odomY;
    const double dist = std::hypot(dx, dy);

    const double curHeading = wrapPi(drive.getHeadingRad());
    double headErr = wrapPi(targetH - curHeading);   // ✅ NOT const

    // ✅ ±180° ambiguity fix
    if (std::fabs(std::fabs(headErr) - M_PI) < TURN_180_FIX_RAD) {
      headErr = 0.0;
    }

    if (dist <= posTolIn && std::fabs(headErr) <= headTolRad) {
      settledMs += 20;
      if (settledMs >= 200) break;
    } else {
      settledMs = 0;
    }

    const double bearing = std::atan2(dy, dx);
    const double moveHeadingForMixer = bearing;

    double moveSpeed = clamp01(std::fabs(distPID.step(dist)));
    moveSpeed = std::min(moveSpeed, clamp01(maxMove));

    double rotSpeed = clamp01(std::fabs(headPID.step(headErr)));
    rotSpeed = std::min(rotSpeed, clamp01(maxRot));

    if (std::fabs(headErr) < ROT_DEADBAND_RAD) {
      rotSpeed = 0.0;
    }

    double turnScale = (dist / DIST_TURN_FADE_IN);
    if (turnScale > 1.0) turnScale = 1.0;
    if (turnScale < 0.2) turnScale = 0.2;
    rotSpeed *= turnScale;

    auto powers = drive.calculateMotorPowers(
        moveHeadingForMixer,
        moveSpeed,
        targetH,
        rotSpeed);

    drive.setMotorPowers(powers);

    if (++printEvery >= 5) {
      printEvery = 0;
      pros::lcd::print(0, "T(%.1f,%.1f) H%.1f", tx, ty, targetHeadingDeg);
      pros::lcd::print(1, "P(%.1f,%.1f) Th%.1f", odomX, odomY, rad2deg(curHeading));
      pros::lcd::print(2, "dx%.1f dy%.1f d%.1f", dx, dy, dist);
      pros::lcd::print(3, "bear%.1f moveH%.1f", rad2deg(bearing), rad2deg(moveHeadingForMixer));
      pros::lcd::print(4, "mSp%.2f rSp%.2f sc%.2f", moveSpeed, rotSpeed, turnScale);
      pros::lcd::print(5, "hErr%.1fdeg set%d", rad2deg(headErr), settledMs);
      pros::lcd::print(6, "LF%4d RF%4d", powers[0], powers[2]);
      pros::lcd::print(7, "LB%4d RB%4d", powers[3], powers[1]);
    }

    pros::Task::delay_until(&t, 20);
  }

  drive.stop();
}




// ------------------------------------------------------------
// Renamed: turnToHeading20164X -> turnToHeading
// ------------------------------------------------------------
void turnToHeading(FieldXDrive& drive,
                   double targetHeadingDeg,
                   double maxRot,
                   int timeoutMs) {
  PID2 headPID{2.4, 0.0, 5.0};
  headPID.reset();

  const double targetH = deg2rad(targetHeadingDeg);
  const uint32_t start = pros::millis();
  const double headTolRad = deg2rad(2.0);

  uint32_t t = pros::millis();
  while ((int)(pros::millis() - start) < timeoutMs) {
    const double curH = drive.getHeadingRad();
    double err = wrapPi(targetH - curH);
    if (std::fabs(err) <= headTolRad) break;

    double rotSpeed = clamp01(std::fabs(headPID.step(err)));
    rotSpeed = std::min(rotSpeed, clamp01(maxRot));

    auto powers = drive.calculateMotorPowers(0.0, 0.0, targetH, rotSpeed);
    drive.setMotorPowers(powers);

    pros::Task::delay_until(&t, 20);
  }

  drive.stop();
}
