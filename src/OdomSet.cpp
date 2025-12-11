#include "main.h"
#include <cmath>

//=============================
// CONSTANTS
//=============================

// Convert degrees <-> radians
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// Wheel diameters
constexpr double VERT_DIAM = 2.75;   // left / right wheels
constexpr double HORIZ_DIAM = 2.00;  // horizontal wheel

// Circumferences
constexpr double VERT_CIRC = VERT_DIAM * M_PI;
constexpr double HORIZ_CIRC = HORIZ_DIAM * M_PI;

// Ticks → inches conversion
constexpr double TPR = 36000.0;
constexpr double VERT_TPI = VERT_CIRC / TPR;
constexpr double HORIZ_TPI = HORIZ_CIRC / TPR;

// TRACKING WHEEL ANGLES
// Left vertical wheel is angled +45°
// Right vertical wheel is angled –45°
constexpr double LEFT_ANGLE  =  45.0 * DEG2RAD;
constexpr double RIGHT_ANGLE = -45.0 * DEG2RAD;

// Precompute sines/cosines
 double L_COS = std::cos(LEFT_ANGLE);
 double L_SIN = std::sin(LEFT_ANGLE);
 double R_COS = std::cos(RIGHT_ANGLE);
 double R_SIN = std::sin(RIGHT_ANGLE);

// Distance from robot center → horizontal wheel (sideways offset)
constexpr double H_OFFSET = 0.0; // set if wheel is NOT centered

//=============================
// GLOBAL STATE
//=============================
double odomX = 0;
double odomY = 0;
double odomTheta = 0;  // radians

double lastL = 0;
double lastR = 0;
double lastH = 0;

double lastHeading = 0;


//=============================
// GET IMU ANGLE (radians)
//=============================
double imuHeading() {
    return IMU.get_rotation() * DEG2RAD;
}


//=============================
// UPDATE ODOMETRY
//=============================
void updateOdom() {

    //--------------------------
    // Read tracking wheels
    //--------------------------
    double L_raw = LVerticalTracker.get_position() * VERT_TPI;
    double R_raw = RVerticalTracker.get_position() * VERT_TPI;
    double H_raw = HorizontalTracker.get_position() * HORIZ_TPI;

    // Compute deltas
    double dL = L_raw - lastL;
    double dR = R_raw - lastR;
    double dH = H_raw - lastH;

    lastL = L_raw;
    lastR = R_raw;
    lastH = H_raw;

    //--------------------------
    // IMU heading change
    //--------------------------
    double heading = imuHeading();
    double dTheta = heading - lastHeading;
    lastHeading = heading;
    odomTheta = heading;

    //--------------------------
    // Convert wheel motion
    // into robot forward/strafe
    //--------------------------

    // Left wheel components
    double dL_forward = dL * L_COS;
    double dL_strafe  = dL * L_SIN;

    // Right wheel components
    double dR_forward = dR * R_COS;
    double dR_strafe  = dR * R_SIN;

    // Combine for forward/strafe
    double forward = (dL_forward + dR_forward) / 2.0;
    double strafe  = (dL_strafe  + dR_strafe ) / 2.0;

    // Add pure horizontal movement
    strafe += dH;

    // Rotation compensation (if wheel not centered)
    strafe -= dTheta * H_OFFSET;

    //--------------------------
    // Rotate into world frame
    //--------------------------
    double cosT = std::cos(odomTheta);
    double sinT = std::sin(odomTheta);

    double dX = forward * sinT + strafe * cosT;
    double dY = forward * cosT - strafe * sinT;

    odomX += dY;      // Y is actually forward
    odomY += -dX;     // X is left/right
}


//=============================
// RESET ODOMETRY
//=============================
void resetOdom() {
    odomX = odomY = 0;
    lastL = lastR = lastH = 0;

    odomTheta = imuHeading();
    lastHeading = odomTheta;

    LVerticalTracker.reset_position();
    RVerticalTracker.reset_position();
    HorizontalTracker.reset_position();
}
