#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// ===========================
//  TRACKING WHEEL CONSTANTS
//  (EDIT THESE FOR YOUR BOT)
// ===========================

// Left vertical tracking wheel (forward)
constexpr double L_DIAM_IN   = 2.25;      // inches
constexpr double L_TICKS_REV = 36000.0;   // sensor ticks per rev

// Right vertical tracking wheel (forward)
constexpr double R_DIAM_IN   = 2.25;
constexpr double R_TICKS_REV = 36000.0;

// Horizontal tracking wheel (strafe)
constexpr double H_DIAM_IN   = 1.25;     
constexpr double H_TICKS_REV = 36000.0;

// Precomputed tick→inch scale factors
constexpr double L_TICKS_TO_IN = (L_DIAM_IN * M_PI) / L_TICKS_REV;
constexpr double R_TICKS_TO_IN = (R_DIAM_IN * M_PI) / R_TICKS_REV;
constexpr double H_TICKS_TO_IN = (H_DIAM_IN * M_PI) / H_TICKS_REV;

// Horizontal tracking wheel offset from robot center (inches).
// + if the wheel is to the LEFT of the center, - if to the RIGHT.
// Measure from robot centerline to the wheel axle.
constexpr double H_SIDE_OFFSET_IN = 3.0;   // <<< MEASURE & TUNE THIS

// ===========================
//  GLOBAL ODOM STATE
// ===========================

// Field coordinates in inches, heading in radians
double odomX      = 0.0;
double odomY      = 0.0;
double odomTheta  = 0.0;   // radians, CCW, 0 = field "forward"

double &xPos   = odomX;
double &yPos   = odomY;
double &theta  = odomTheta;

// Previous wheel distances (inches)
static double lastL = 0.0;
static double lastR = 0.0;
static double lastH = 0.0;

// Previous heading (radians)
static double lastHeading = 0.0;

// Helper: IMU heading in radians (wrap to [-pi, pi])
static double getHeadingRad() {
  double deg = IMU.get_rotation();          // [-180, 180] typically
  // wrap just in case
  while (deg > 180)  deg -= 360;
  while (deg < -180) deg += 360;
  return deg * M_PI / 180.0;
}

// ===========================
//  ODOMETRY API
// ===========================

// Call once at start of auton (and whenever you want to reset pose)
void resetOdom(double xInches, double yInches, double headingDeg) {
    odomX = xInches;
    odomY = yInches;
    odomTheta = headingDeg * M_PI / 180.0;

    lastHeading = odomTheta;

    lastL = 0.0;
    lastR = 0.0;
    lastH = 0.0;

    LVerticalTracker.reset_position();
    RVerticalTracker.reset_position();
    HorizontalTracker.reset_position();
}

// Overload for zero pose
void resetOdom() {
    resetOdom(0.0, 0.0, 0.0);
}

// Call this in a 10–20ms loop (opcontrol task or auton task)
void updateOdom() {
    // 1) Read current ticks and convert to inches
    double L_in = LVerticalTracker.get_position() * L_TICKS_TO_IN;
    double R_in = RVerticalTracker.get_position() * R_TICKS_TO_IN;
    double H_in = HorizontalTracker.get_position() * H_TICKS_TO_IN;

    // 2) Compute deltas since last update
    double dL = L_in - lastL;
    double dR = R_in - lastR;
    double dH = H_in - lastH;

    lastL = L_in;
    lastR = R_in;
    lastH = H_in;

    // 3) Heading from IMU
    double heading = getHeadingRad();
    double dTheta  = heading - lastHeading;
    odomTheta      = heading;
    lastHeading    = heading;

    // 4) Robot-frame translation
    // Forward is average of L & R vertical wheels
    double forward = (dL + dR) / 2.0;

    // Horizontal wheel measures strafe + rotation effect.
    // Compensate for wheel offset:
    double strafe = dH - dTheta * H_SIDE_OFFSET_IN;

    // 5) Rotate robot-frame delta into field frame
    double cosT = std::cos(odomTheta);
    double sinT = std::sin(odomTheta);

    // Coordinate convention:
    //  - odomY: forward on field
    //  - odomX: left on field
    double dX =  forward * sinT + strafe * cosT;
    double dY =  forward * cosT - strafe * sinT;

    odomX += dX;
    odomY += dY;
}
