#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// Odom state from OdomSet.cpp
extern double odomX;
extern double odomY;
extern double odomTheta;   // radians

// ---- Motor aliases (from subsystems.hpp) ----
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

// ===========================
//  DRIVE HELPERS
// ===========================

static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Simple slew-rate limiter
static double slewRate(double target, double current, double maxDelta) {
    double diff = target - current;
    if (diff >  maxDelta) diff =  maxDelta;
    if (diff < -maxDelta) diff = -maxDelta;
    return current + diff;
}

void StopBase() {
    FL1.move(0); FL2.move(0);
    FR1.move(0); FR2.move(0);
    BL1.move(0); BL2.move(0);
    BR1.move(0); BR2.move(0);
}

// ===========================
//  DRIVE-TO-POINT PID (X-DRIVE)
// ===========================
//
//  targetX, targetY: field coords in inches (same frame as odomX/odomY)
//  targetHeadingDeg: desired field heading in degrees (0 = forward)
//  maxSpeed: max motor command magnitude (e.g. 100)
//  slew: max change per cycle in motor power for smoothness
//
void DriveToPoint_PID(double targetX,
                      double targetY,
                      double targetHeadingDeg,
                      double maxSpeed,
                      double slew) {

    // ---- XY PID gains 
    const double kP_xy = 0.9;
    const double kI_xy = 0.003;
    const double kD_xy = 2.5;

    // ---- Rotation PID gains ----
    const double kP_rot = 3.0;
    const double kI_rot = 0.0;
    const double kD_rot = 0.4;

    // PID state
    double fwdErr = 0, strafeErr = 0, rotErr = 0;
    double prevFwdErr = 0, prevStrafeErr = 0, prevRotErr = 0;
    double fwdInt = 0, strafeInt = 0, rotInt = 0;

    // Motor power with slew
    double fl = 0, fr = 0, bl = 0, br = 0;

    // Heading target in radians
    double targetHeadingRad = targetHeadingDeg * M_PI / 180.0;

    // Exit conditions
    const double posTolIn   = 0.5;           // inches
    const double angTolRad  = 1.5 * M_PI/180.0; // ~1.5 deg
    const int    settleTimeMs = 150;         // must be in-tolerance this long
    int          inTolTime = 0;

    // Optional safety timeout
    const int maxRunTimeMs = 4000;
    int startTime = pros::millis();

    while (true) {
        // --- Update odometry (must be called regularly somewhere) ---
        updateOdom();   // If you call updateOdom in a separate task, remove this line.

        // --- Field error ---
        double dx = targetX - odomX;   // (+X left)
        double dy = targetY - odomY;   // (+Y forward)

        double dist = std::sqrt(dx*dx + dy*dy);

        // --- Heading error (wrap) ---
        rotErr = targetHeadingRad - odomTheta;
        while (rotErr >  M_PI) rotErr -= 2*M_PI;
        while (rotErr < -M_PI) rotErr += 2*M_PI;

        // Check convergence
        bool posOK = dist   < posTolIn;
        bool angOK = std::fabs(rotErr) < angTolRad;

        if (posOK && angOK) {
            inTolTime += 15;
            if (inTolTime >= settleTimeMs) break;
        } else {
            inTolTime = 0;
        }

        if (pros::millis() - startTime > maxRunTimeMs) {
            // timeout – don't get stuck forever
            break;
        }

        // --- Convert field error to robot-frame error ---
        double cosT = std::cos(odomTheta);
        double sinT = std::sin(odomTheta);

        // Robot-frame forward (Y) and strafe (X)
        fwdErr    =  dx * sinT + dy * cosT;
        strafeErr =  dx * cosT - dy * sinT;

        // === PID for forward & strafe ===
        fwdInt    += fwdErr;
        strafeInt += strafeErr;

        double fwdDer    = fwdErr    - prevFwdErr;
        double strafeDer = strafeErr - prevStrafeErr;

        prevFwdErr    = fwdErr;
        prevStrafeErr = strafeErr;

        double fwdPower =
            kP_xy * fwdErr +
            kI_xy * fwdInt +
            kD_xy * fwdDer;

        double strafePower =
            kP_xy * strafeErr +
            kI_xy * strafeInt +
            kD_xy * strafeDer;

        // === PID for rotation ===
        rotInt += rotErr;
        double rotDer = rotErr - prevRotErr;
        prevRotErr = rotErr;

        double rotPower =
            kP_rot * rotErr +
            kI_rot * rotInt +
            kD_rot * rotDer;

        // Clamp base powers
        fwdPower    = clamp(fwdPower,    -maxSpeed, maxSpeed);
        strafePower = clamp(strafePower, -maxSpeed, maxSpeed);
        rotPower    = clamp(rotPower,    -maxSpeed, maxSpeed);

        // --- X-drive mixing (robot-centric) ---
        double targetFL = fwdPower + strafePower + rotPower;
        double targetFR = fwdPower - strafePower - rotPower;
        double targetBL = fwdPower - strafePower + rotPower;
        double targetBR = fwdPower + strafePower - rotPower;

        // --- Slew limit each motor ---
        fl = slewRate(targetFL, fl, slew);
        fr = slewRate(targetFR, fr, slew);
        bl = slewRate(targetBL, bl, slew);
        br = slewRate(targetBR, br, slew);

        // --- Send to motors ---
        FL1.move(fl);  FL2.move(fl);
        BL1.move(bl);  BL2.move(bl);
        FR1.move(fr);  FR2.move(fr);
        BR1.move(br);  BR2.move(br);

        pros::delay(15);
    }

    StopBase();
}
