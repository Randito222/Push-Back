// OdomSet.cpp
#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// ---------- CONSTANTS ----------
const double TICKS_PER_REV = 36000.0;       // pros::Rotation: 36000 ticks / rev
const double WHEEL_DIAMETER = 2.75;         // tracking wheel diameter (inches)
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TICKS_TO_INCH = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;

// Distance of the sideways tracking wheel from the robot center (inches).
// Set this correctly for best results. Start with rough guess and tune.
const double SIDE_OFFSET = 2.5;            // if your Horzintal wheel is centered, leave 0

// ---------- GLOBAL ODOM STATE ----------
double odomX = 0.0;        // inches
double odomY = 0.0;        // inches
double odomTheta = 0.0;    // radians (IMU-based)

// Previous encoder positions (in inches)
double lastL = 0.0;
double lastR = 0.0;
double lastH = 0.0;

// Previous heading from IMU (radians)
double lastHeading = 0.0;

// ---------- INIT / RESET ----------
void initOdom() {
  IMU.reset();
  while (IMU.is_calibrating()) {
    pros::delay(10);
  }

  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  lastL = 0.0;
  lastR = 0.0;
  lastH = 0.0;

  odomX = 0.0;
  odomY = 0.0;
  odomTheta = IMU.get_rotation() * M_PI / 180.0;
  lastHeading = odomTheta;
}

void resetOdom(double xInches, double yInches, double headingDeg) {
  odomX = xInches;
  odomY = yInches;
  odomTheta = headingDeg * M_PI / 180.0;
  lastHeading = odomTheta;

  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  lastL = 0.0;
  lastR = 0.0;
  lastH = 0.0;
}

// ---------- GET HEADING ----------
double getHeadingRad() {
  return IMU.get_rotation() * M_PI / 180.0;
}

// ---------- ODOMETRY UPDATE ----------
void updateOdom() {

    // ===============================
    // Read encoder positions (ticks → inches)
    // ===============================
    double L = LVerticalTracker.get_position() * TICKS_TO_INCH;     // Left vertical wheel
    double R = RVerticalTracker.get_position() * TICKS_TO_INCH;     // Right vertical wheel
    double H = HorizontalTracker.get_position() * TICKS_TO_INCH;    // Strafe wheel

    // Compute deltas since last loop
    
    double dL = L - lastL;
    double dR = R - lastR;
    double dH = H - lastH;

    lastL = L;
    lastR = R;
    lastH = H;

    // ===============================
    // Update heading from IMU
    // ===============================
    double heading = getHeadingRad();    // radians
    double dTheta = heading - lastHeading;
    odomTheta = heading;
    lastHeading = heading;

    // ===============================
    // Robot-frame movement
    // Vertical trackers measure FORWARD
    // Horizontal tracker measures STRAFE
    // ===============================
    double forward = (dL + dR) / 2.0;  // average of L & R
    double strafe  = dH;               // horizontal movement

    //If your horizontal tracker is offset from center:
    strafe -= dTheta * SIDE_OFFSET;

    // ===============================
    // Convert robot-frame movement into field-frame movement
    // ===============================
    double cosT = cos(odomTheta);
    double sinT = sin(odomTheta);

    double dX =  forward * sinT + strafe * cosT;
    double dY =  forward * cosT - strafe * sinT;

    // ===============================
    // Apply to global coordinates
    // ===============================
    odomX += dX;
    odomY += dY;
}