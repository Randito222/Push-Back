#include "OdomSet.hpp"
#include "subsystems.hpp"
#include "pros/apix.h"
#include <cmath>

// =============================
// Wheel geometry
// =============================
constexpr double VERT_DIAM_IN = 2.75;   // vertical tracking wheels
constexpr double HORZ_DIAM_IN = 2.00;   // horizontal tracking wheel
constexpr double TICKS_REV    = 360.0;  // Rotation sensor: degrees per revolution

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// =============================
// Odom State
// =============================
double odomX = 0.0;
double odomY = 0.0;
double odomTheta = 0.0;

// =============================
// Helpers
// =============================
static double wheelCirc(double diamIn) {
  return M_PI * diamIn;
}

static double ticksToInches(double ticksDeg, double diamIn) {
  // Rotation sensor returns degrees
  return (ticksDeg / 360.0) * wheelCirc(diamIn);
}

static double wrapRad(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

// =============================
// Last readings
// =============================
static double lastVL = 0.0;
static double lastVR = 0.0;
static double lastH  = 0.0;
static double lastHeading = 0.0;

// =============================
// Update
// =============================
void updateOdom() {
  // Read sensors (apply sign flips here ONCE if needed)
  // Forward should be + for BOTH vertical wheels after sign flips.
  const double vlNow = ticksToInches(-LVerticalTracker.get_position(), VERT_DIAM_IN);
  const double vrNow = ticksToInches(RVerticalTracker.get_position(), VERT_DIAM_IN);
  const double hNow  = ticksToInches(HorizontalTracker.get_position(), HORZ_DIAM_IN);

  double heading = IMU.get_rotation() * DEG2RAD;  // degrees -> radians
  if (!std::isfinite(heading)) return;

  // Deltas
  const double dVL = vlNow - lastVL;
  const double dVR = vrNow - lastVR;
  const double dH  = hNow  - lastH;

  double dTheta = wrapRad(heading - lastHeading);

  lastVL = vlNow;
  lastVR = vrNow;
  lastH  = hNow;
  lastHeading = heading;

  // =============================
  // Robot-relative motion
  // =============================
  // Vertical wheels measure Y only
  const double dY_robot = (dVL + dVR) * 0.5;

  // Horizontal wheel measures X only
  const double dX_robot = dH;

  // =============================
  // Robot -> Field transform using current heading
  // =============================
  const double sinH = std::sin(heading);
  const double cosH = std::cos(heading);

  odomX += dX_robot * cosH - dY_robot * sinH;
  odomY += dX_robot * sinH + dY_robot * cosH;

  odomTheta = heading;
}

void resetOdom(double x, double y, double headingDeg) {
  odomX = x;
  odomY = y;
  odomTheta = headingDeg * DEG2RAD;

  IMU.set_rotation(headingDeg);

  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  lastVL = 0.0;
  lastVR = 0.0;
  lastH  = 0.0;
  lastHeading = odomTheta;
}

void printOdom() {
  pros::lcd::print(0, "X: %.2f", odomX);
  pros::lcd::print(1, "Y: %.2f", odomY);
  pros::lcd::print(2, "H: %.1f", odomTheta * RAD2DEG);
}
