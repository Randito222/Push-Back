#include "OdomSet.hpp"
#include "subsystems.hpp"
#include "pros/apix.h"
#include <cmath>

// =============================
<<<<<<< HEAD
// Wheel geometry
// =============================
constexpr double VERT_DIAM  = 2.75;   // angled wheels
constexpr double HORZ_DIAM  = 2.00;   // horizontal wheel
constexpr double TICKS_REV  = 360.0;

=======
// Wheel geometry 
// =============================
constexpr double VERT_DIAM_IN = 2.75;
constexpr double HORZ_DIAM_IN = 2.00;

// =============================
// ODOM OFFSETS 
// =============================
// Distance between the LEFT and RIGHT vertical tracking wheels (center-to-center).
// Measure in inches.
constexpr double TRACK_WIDTH_IN = 6.0;   // <-- CHANGE THIS

// Horizontal wheel offset from robot center (inches):
// + if the horizontal wheel is IN FRONT of the robot center
// - if the horizontal wheel is BEHIND the robot center
constexpr double H_OFFSET_IN = -3.0;       // <-- CHANGE THIS (0 only if it's exactly centered)

// =============================
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
constexpr double DEG2RAD = M_PI / 180.0;

<<<<<<< HEAD
// Angles of the vertical wheels
constexpr double LEFT_ANGLE  = -45.0 * DEG2RAD;
constexpr double RIGHT_ANGLE =  45.0 * DEG2RAD;

// Offset: center of robot → horizontal wheel
constexpr double HORZ_OFFSET = 4.0;   // inches (MEASURE THIS)

// =============================
// Odometry State (DEFINITION)
// =============================
double odomX = 0.0;       // inches
double odomY = 0.0;       // inches
double odomTheta = 0.0;   // radians

// =============================
// Utility Functions
// =============================
static double wheelCirc(double d) {
    return M_PI * d;
}

static double ticksToInches(double ticks, double diam) {
    return (ticks / TICKS_REV) * wheelCirc(diam);
}

// =============================
// Last Sensor Values
=======
// =============================
// Odom State (global)
// =============================
double odomX = 0.0;      // field X (right +)
double odomY = 0.0;      // field Y (forward +)
double odomTheta = 0.0;  // heading radians

// =============================
// Helpers
// =============================
static double wheelCirc(double diamIn) {
  return M_PI * diamIn;
}

// pros::Rotation get_position() returns centidegrees (cdeg)
// 36000 cdeg per revolution
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
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
// =============================
static double lastVL = 0.0;
static double lastVR = 0.0;
static double lastH  = 0.0;
static double lastHeading = 0.0;

// =============================
<<<<<<< HEAD
// Odometry Update
=======
// Update
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
// =============================
void updateOdom() {
  if (IMU.is_calibrating()) return;

<<<<<<< HEAD
    // --- Read sensors ---
    double vlNow = ticksToInches(-LVerticalTracker.get_position(),  VERT_DIAM);
    double vrNow = ticksToInches(RVerticalTracker.get_position(),  VERT_DIAM);
    double hNow  = ticksToInches(HorizontalTracker.get_position(), HORZ_DIAM);

    double heading = IMU.get_rotation() * DEG2RAD;

    // --- Deltas ---
    double dVL = vlNow - lastVL;
    double dVR = vrNow - lastVR;
    double dH  = hNow  - lastH;
    double dTheta = heading - lastHeading;

    lastVL = vlNow;
    lastVR = vrNow;
    lastH  = hNow;
    lastHeading = heading;

    // =============================
    // Robot-relative motion
    // =============================
    // Vector reconstruction from ±45° wheels
    double dX = (dVL * cos(LEFT_ANGLE) + dVR * cos(RIGHT_ANGLE)) / 2.0;
    double dY = (dVL * sin(LEFT_ANGLE) + dVR * sin(RIGHT_ANGLE)) / 2.0;

    // Horizontal wheel rotation compensation
    if (fabs(dTheta) > 1e-6) {
        dX += dTheta * HORZ_OFFSET;
    }

    // =============================
    // Field-relative update
    // =============================
    double sinH = sin(heading);
    double cosH = cos(heading);

    odomX += dX * cosH - dY * sinH;
    odomY += dX * sinH + dY * cosH;

    odomTheta = heading;
}

// =============================
// Odometry Task
// =============================
void odomTask() {
    while (true) {
        updateOdom();
        pros::delay(10);
    }
}

// =============================
// Reset Odometry
// =============================
void resetOdom() {

    odomX = 0.0;
    odomY = 0.0;
    odomTheta = 0.0;

    IMU.set_rotation(0);

    LVerticalTracker.reset_position();
    RVerticalTracker.reset_position();
    HorizontalTracker.reset_position();

    lastVL = lastVR = lastH = 0.0;
    lastHeading = 0.0;
}

void printOdom() {
    pros::lcd::print(0, "X: %.2f", odomX);
    pros::lcd::print(1, "Y: %.2f", odomY);
    pros::lcd::print(2, "H: %.2f", odomTheta);
=======
  // Read sensors (apply sign flips here ONCE)
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

  // =============================
  // Robot-relative deltas
  // =============================
  // Forward/back is average of vertical wheels
  const double dY_robot = (dVL + dVR) * 0.5;

  // Strafe is horizontal wheel minus the amount caused purely by turning
  // Turning causes the horizontal wheel to roll: arc = dTheta * offset
  const double dX_robot = dH - (dTheta * H_OFFSET_IN);

  // Compare IMU turn vs wheel turn:
  // const double dTheta_wheels = (dVR - dVL) / TRACK_WIDTH_IN;

  // =============================
  // Robot -> Field transform (use mid-heading)
  // =============================
  const double midHeading = heading - (dTheta * 0.5);
  const double sinH = std::sin(midHeading);
  const double cosH = std::cos(midHeading);

  odomX += dX_robot * cosH - dY_robot * sinH;
  odomY += dX_robot * sinH + dY_robot * cosH;

  odomTheta = heading;
}

// =============================
// Reset
// =============================
void resetOdom() {
  odomX = 0.0;
  odomY = 0.0;
  odomTheta = 0.0;

  IMU.set_rotation(0);

  // Clear rotation sensors
  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  lastVL = 0.0;
  lastVR = 0.0;
  lastH  = 0.0;
  lastHeading = 0.0;
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
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