#include "OdomSet.hpp"
#include "subsystems.hpp"
#include "pros/apix.h"
#include <cmath>

// =============================
// Wheel geometry
// =============================
constexpr double VERT_DIAM  = 2.75;   // angled wheels
constexpr double HORZ_DIAM  = 2.00;   // horizontal wheel
constexpr double TICKS_REV  = 360.0;

constexpr double DEG2RAD = M_PI / 180.0;

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
// =============================
static double lastVL = 0.0;
static double lastVR = 0.0;
static double lastH  = 0.0;
static double lastHeading = 0.0;

// =============================
// Odometry Update
// =============================
void updateOdom() {

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
}
