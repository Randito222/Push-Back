#include "main.h"

// -------- CONSTANTS --------
const double TICKS_TO_INCH = (1.0 / 360.0) * 2.75 * M_PI;  
// 360 ticks per rev, 2.75" tracking wheel

// Global odom variables
double xPos = 0;
double yPos = 0;
double theta = 0;

// Previous encoder values
double lastL = 0;
double lastR = 0;
double lastV = 0;

void initOdom() {
    IMU.reset();
    while (IMU.is_calibrating()) pros::delay(10);

    // Reset trackers
    LHorizontalTracker.reset_position();
    RHorizontalTracker.reset_position();
    VerticalTracker.reset_position();
}

double getHeadingRad() {
    double headingDeg = IMU.get_rotation();
    return headingDeg * M_PI / 180.0;
}

void updateOdom() {

    // Read all sensors
    double L = LHorizontalTracker.get_position() * TICKS_TO_INCH;
    double R = RHorizontalTracker.get_position() * TICKS_TO_INCH;
    double V = VerticalTracker.get_position()  * TICKS_TO_INCH;

    // Compute delta
    double dL = L - lastL;
    double dR = R - lastR;
    double dV = V - lastV;

    // Save new values
    lastL = L;
    lastR = R;
    lastV = V;

    // IMU-synced heading
    theta = getHeadingRad();

    // Forward + strafe local movement
    double forward = (dL + dR) / 2.0;
    double strafe  = dV;

    // Rotate into field global coordinates
    xPos += forward * cos(theta) - strafe * sin(theta);
    yPos += forward * sin(theta) + strafe * cos(theta);
}