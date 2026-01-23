#include "OdomSet.hpp"
#include "subsystems.hpp"
#include "pros/apix.h"
#include <cmath>

// =============================
// Wheel geometry
// =============================
constexpr double VERT_DIAM_IN = 2.75;
constexpr double HORZ_DIAM_IN = 2.00;

// =============================
// ODOM OFFSETS
// =============================
constexpr double TRACK_WIDTH_IN = 6.0;   // CHANGE THIS (inches, center-to-center)
constexpr double H_OFFSET_IN    = -3.0;  // CHANGE THIS (inches, +front / -back)

// =============================
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// =============================
// Odom State (global)
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

// Rotation get_position() returns centidegrees (cdeg): 36000 cdeg per rev
static double rotCdegToInches(double cdeg, double diamIn) {
  return (cdeg / 36000.0) * wheelCirc(diamIn);
}

static double wrapRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
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
  if (IMU.is_calibrating()) return;

  // Read sensors (apply sign flips once)
  // Forward should be + for BOTH vertical wheels.
  const double vlNow = rotCdegToInches(-LVerticalTracker.get_position(), VERT_DIAM_IN);
  const double vrNow = rotCdegToInches( RVerticalTracker.get_position(), VERT_DIAM_IN);
  const double hNow  = rotCdegToInches( HorizontalTracker.get_position(), HORZ_DIAM_IN);

  const double heading = IMU.get_rotation() * DEG2RAD;
  if (!std::isfinite(heading)) return;

  // Deltas
  const double dVL = vlNow - lastVL;
  const double dVR = vrNow - lastVR;
  const double dH  = hNow  - lastH;

  const double dTheta = wrapRad(heading - lastHeading);

  // Save state
  lastVL = vlNow;
  lastVR = vrNow;
  lastH  = hNow;
  lastHeading = heading;

  // Robot-relative deltas
  const double dY_robot = (dVL + dVR) * 0.5;
  const double dX_robot = dH - (dTheta * H_OFFSET_IN);

  // Robot -> Field transform using mid-heading
  const double midHeading = heading - (dTheta * 0.5);
  const double sinH = std::sin(midHeading);
  const double cosH = std::cos(midHeading);

  // Convention: heading=0 means facing +Y, so this matches your drive code
  odomX += dX_robot * cosH - dY_robot * sinH;
  odomY += dX_robot * sinH + dY_robot * cosH;

  odomTheta = heading;
}

// =============================
// Reset helpers
// =============================
void resetOdomPoseRad(double x_in, double y_in, double thetaRad) {
  odomX = x_in;
  odomY = y_in;
  odomTheta = wrapRad(thetaRad);

  // Set IMU to match pose heading (degrees)
  IMU.set_rotation(odomTheta * RAD2DEG);

  // Clear rotation sensors
  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  // Reset last readings to match now
  lastVL = 0.0;
  lastVR = 0.0;
  lastH  = 0.0;
  lastHeading = odomTheta;
}

void resetOdomPose(double x_in, double y_in, double thetaDeg) {
  resetOdomPoseRad(x_in, y_in, thetaDeg * DEG2RAD);
}

void resetOdom() {
  resetOdomPoseRad(0.0, 0.0, 0.0);
}

// =============================
// Print
// =============================
void printOdom() {
  pros::lcd::print(0, "X: %.2f", odomX);
  pros::lcd::print(1, "Y: %.2f", odomY);
  pros::lcd::print(2, "H: %.1f deg", odomTheta * RAD2DEG);
}

// =============================
// Task
// =============================
void odomTask(void*) {
  while (true) {
    updateOdom();
    pros::delay(10);
  }
}
