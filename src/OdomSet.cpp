#pragma once
#include "pros/apix.h"
#include "subsystems.hpp"
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

// Offsets (center of robot → wheel contact)
constexpr double HORZ_OFFSET = 4.0;   // inches (measure this)

struct OdomState {
    double x = 0;
    double y = 0;
    double headingDeg = 0;
};

OdomState Myodom;

// =============================
// Utility Functions
// =============================

double wheelCirc(double d) {
    return M_PI * d;
}

double ticksToInches(double ticks, double diam) {
    return (ticks / TICKS_REV) * wheelCirc(diam);
}

// =============================
// Last Sensor Values
// =============================

double lastVL = 0;
double lastVR = 0;
double lastH  = 0;
double lastHeading = 0;


void updateOdometry() {

    // --- Read sensors ---
    double vlNow = ticksToInches(LVerticalTracker.get_position(),  VERT_DIAM);
    double vrNow = ticksToInches(RVerticalTracker.get_position(), VERT_DIAM);
    double hNow  = ticksToInches(HorizontalTracker.get_position(),     HORZ_DIAM);

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
    // Solve robot-relative motion
    // =============================

    // Vertical wheels → vector reconstruction
    // d = dx*cos(θ) + dy*sin(θ)

    double dX = (dVL * cos(LEFT_ANGLE) + dVR * cos(RIGHT_ANGLE)) / 2.0;
    double dY = (dVL * sin(LEFT_ANGLE) + dVR * sin(RIGHT_ANGLE)) / 2.0;

    // Horizontal wheel correction for rotation
    if (fabs(dTheta) > 1e-6) {
        dX += dTheta * HORZ_OFFSET;
    }

    // =============================
    // Rotate into field coordinates
    // =============================
    double sinH = sin(heading);
    double cosH = cos(heading);

    Myodom.x += dX * cosH - dY * sinH;
    Myodom.y += dX * sinH + dY * cosH;

    Myodom.headingDeg = heading / DEG2RAD;
}

void odomTask(void*) {
    while (true) {
        updateOdometry();
        pros::delay(10);
    }
}

void resetOdom(double x = 0, double y = 0, double headingDeg = 0) {

    Myodom.x = x;
    Myodom.y = y;
    Myodom.headingDeg = headingDeg;

    IMU.set_rotation(headingDeg);

    LVerticalTracker.reset_position();
    RVerticalTracker.reset_position();
    HorizontalTracker.reset_position();

    lastVL = lastVR = lastH = 0;
    lastHeading = headingDeg * DEG2RAD;
}

void printOdom() {
    pros::lcd::print(0, "X: %.2f", Myodom.x);
    pros::lcd::print(1, "Y: %.2f", Myodom.y);
    pros::lcd::print(2, "H: %.2f", Myodom.headingDeg);
}




