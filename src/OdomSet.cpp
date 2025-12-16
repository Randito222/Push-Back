#include "OdomSet.hpp"
#include "subsystems.hpp"
#include <cmath>

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double TPR = 36000.0;
constexpr double VERT_TPI  = (2.75 * M_PI) / TPR;
constexpr double HORIZ_TPI = (2.0  * M_PI) / TPR;

double odomX = 0;
double odomY = 0;
double odomTheta = 0;

static double lastL = 0, lastR = 0, lastH = 0, lastHeading = 0;

static double imuHeading() {
  return IMU.get_rotation() * DEG2RAD;
}

void updateOdom() {
  double L = -LVerticalTracker.get_position() * VERT_TPI;
  double R =  RVerticalTracker.get_position() * VERT_TPI;
  double H =  HorizontalTracker.get_position() * HORIZ_TPI;

  double dL = L - lastL;
  double dR = R - lastR;
  double dH = H - lastH;

  lastL = L; lastR = R; lastH = H;

  double heading = imuHeading();
  double dTheta = heading - lastHeading;
  lastHeading = heading;
  odomTheta = heading;

  double forward = (dL + dR) / 2.0;
  double strafe  = dH;

  double c = cos(odomTheta);
  double s = sin(odomTheta);

  odomX += -(forward * s + strafe * c);
  odomY +=  (forward * c - strafe * s);
}

void resetOdom() {
  odomX = odomY = 0;
  lastL = lastR = lastH = 0;
  odomTheta = lastHeading = imuHeading();

  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();
}

void odomTask() {
  while (true) {
    updateOdom();
    pros::delay(20);
  }
}
