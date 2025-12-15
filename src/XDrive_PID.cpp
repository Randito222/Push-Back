#include "main.h"
#include "subsystems.hpp"

#include <cmath>

// Globals from OdomSet.cpp
extern double odomX, odomY, odomTheta;

// Motor aliases
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

// ------------------------
// Utility helpers
// ------------------------
// Convert degrees <-> radians
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

double clamp(double v, double lo, double hi) {
    return (v < lo ? lo : (v > hi ? hi : v));
}

double slewRate(double target, double current, double rate) {
    double diff = target - current;
    if (std::fabs(diff) > rate)
        return current + rate * (diff > 0 ? 1 : -1);
    return target;
}

void StopBase(){
    FL1.move(0); FL2.move(0);
    FR1.move(0); FR2.move(0);
    BL1.move(0); BL2.move(0);
    BR1.move(0); BR2.move(0);
}

//===================================================
// PERFECT X-DRIVE PID WITH ODOMETRY
//===================================================
void DriveToPoint_PID(double targetX,
                      double targetY,
                      double targetHeadingDeg,
                      double maxSpeed,
                      double slew)
{
    // Translation PID
    const double kP = 4.2;
    const double kI = 0.002;
    const double kD = 0.3;

    // Rotation PID
    const double kP_rot = 2.0;
    const double kI_rot = 0.0;
    const double kD_rot = 0.4;

    double targetHeading = targetHeadingDeg * DEG2RAD;

    // PID state
    double fwdErr, strafeErr, rotErr;
    double prevFwd = 0, prevStrafe = 0, prevRot = 0;
    double fwdInt = 0, strafeInt = 0, rotInt = 0;

    // Motor powers
    double fl = 0, fr = 0, bl = 0, br = 0;

    int settleTimer = 0;

    while (true) {

        updateOdom();

        //--------------------------
        // Compute XY ERROR
        //--------------------------
        double dx = targetX - odomX;
        double dy = targetY - odomY;

        double dist = sqrt(dx*dx + dy*dy);

        //--------------------------
        // Compute rotation error
        //--------------------------
        rotErr = targetHeading - odomTheta;
        while (rotErr >  M_PI) rotErr -= 2*M_PI;
        while (rotErr < -M_PI) rotErr += 2*M_PI;

        //--------------------------
        // Robot-frame conversion
        //--------------------------
        double cosT = cos(-odomTheta);
        double sinT = sin(-odomTheta);

        fwdErr    = dy * cosT - dx * sinT;
        strafeErr = dy * sinT + dx * cosT;

        //--------------------------
        // Check settling
        //--------------------------
        if (fabs(fwdErr) < 0.5 &&
            fabs(strafeErr) < 0.5 &&
            fabs(rotErr * RAD2DEG) < 1.0)
        {
            settleTimer += 15;
            if (settleTimer > 200) break;
        }
        else settleTimer = 0;

        //--------------------------
        // PID calculations
        //--------------------------
        fwdInt += fwdErr;
        strafeInt += strafeErr;
        rotInt += rotErr;

        double fwdDer = fwdErr - prevFwd;     prevFwd = fwdErr;
        double strafeDer = strafeErr - prevStrafe; prevStrafe = strafeErr;
        double rotDer = rotErr - prevRot;     prevRot = rotErr;

        double fwdPower =
            kP*fwdErr + kI*fwdInt + kD*fwdDer;

        double strafePower =
            kP*strafeErr + kI*strafeInt + kD*strafeDer;

        double rotPower =
            kP_rot*rotErr + kI_rot*rotInt + kD_rot*rotDer;

        fwdPower    = clamp(fwdPower,    -maxSpeed, maxSpeed);
        strafePower = clamp(strafePower, -maxSpeed, maxSpeed);
        rotPower    = clamp(rotPower,    -maxSpeed, maxSpeed);

        //--------------------------
        // X-Drive Mixing
        //--------------------------
        double s = -strafePower;   // FIX STRAFE DIRECTION

        double tFL = fwdPower + s + rotPower;
        double tFR = fwdPower - s - rotPower;
        double tBL = fwdPower - s + rotPower;
        double tBR = fwdPower + s - rotPower;

        //--------------------------
        // Slew-rate limit
        //--------------------------
        fl = slewRate(tFL, fl, slew);
        fr = slewRate(tFR, fr, slew);
        bl = slewRate(tBL, bl, slew);
        br = slewRate(tBR, br, slew);

        //--------------------------
        // Send to motors
        //--------------------------
        FL1.move(fl);  FL2.move(fl);
        FR1.move(fr);  FR2.move(fr);
        BL1.move(bl);  BL2.move(bl);
        BR1.move(br);  BR2.move(br);

        pros::lcd::print(0, "Odom X:%.2f  Y:%.2f", odomX, odomY);
        pros::lcd::print(1, "dx:%.2f  dy:%.2f", dx, dy);
        pros::lcd::print(2, "fwdErr:%.2f  strErr:%.2f", fwdErr, strafeErr);
        pros::lcd::print(3, "rotErr:%.2f deg", rotErr * RAD2DEG);

        pros::lcd::print(4, "fwdPow:%.2f strPow:%.2f rotPow:%.2f",
            fwdPower, strafePower, rotPower);

        pros::lcd::print(5, "FL:%.1f FR:%.1f", fl, fr);
        pros::lcd::print(6, "BL:%.1f BR:%.1f", bl, br);

        master.print(0, 0, "X%.1f Y%.1f", odomX, odomY);
        master.print(1, 0, "F%.1f S%.1f", fwdErr, strafeErr);
        master.print(2, 0, "Rot%.1f", rotErr * RAD2DEG);


        pros::delay(15);
    }

    // Stop robot at the end
    FL1.move(0); FL2.move(0);
    FR1.move(0); FR2.move(0);
    BL1.move(0); BL2.move(0);
    BR1.move(0); BR2.move(0);
}
