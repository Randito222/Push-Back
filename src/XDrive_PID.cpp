#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// ---- Tracking Wheel Constants ----
const double TICKS_PER_REV = 36000.0; 
const double WHEEL_DIAMETER = 2.75; 
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 10.0;   // Distance between left/right horizontal wheels

// ---- Global odometry state ----
double globalX = 0;
double globalY = 0;
double globalTheta = 0;  // degrees

// ---- Motor definitions ----
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

// ---- Helpers ----
double ticksToInches(double ticks) {
    return (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
}

double KeepInRange(double value, double minValue, double maxValue) {
    if(value < minValue) value = minValue;
    if(value > maxValue) value = maxValue;
    return value;
}

// ---- Slew rate function ----
double slewRate(double target, double current, double rate) {
    if(current < target) current += rate;
    else if(current > target) current -= rate;
    
    // Snap to target if close
    if(fabs(target - current) < rate) current = target;
    return current;
}

// ---- Stop base ----
void StopBase() {
    FL1.move(0); 
    FL2.move(0);
    BL1.move(0); 
    BL2.move(0);
    FR1.move(0); 
    FR2.move(0);
    BR1.move(0); 
    BR2.move(0);
}



// ---- Turn PID with slew rate ----
void TurnToAngle_PID(double targetAngle, double maxSpeed = 80, double exitError = 1.0, double slew = 2.0) {
    IMU.set_rotation(0);
    double finalTarget = IMU.get_rotation() + targetAngle;

    double kP = 2.0;
    double kI = 0.0;
    double kD = 0.3;

    double integral = 0;
    double prevError = 0;

    double fl = 0, bl = 0, fr = 0, br = 0; // slew-rate applied

    while(true) {
        double error = finalTarget - IMU.get_rotation();
        while(error > 180) error -= 360;
        while(error < -180) error += 360;

        if(fabs(error) <= exitError) break;

        integral += error;
        double derivative = error - prevError;
        prevError = error;

        double rotPower = kP*error + kI*integral + kD*derivative;
        rotPower = KeepInRange(rotPower, -maxSpeed, maxSpeed);

        // Apply slew rate
        fl = slewRate(rotPower, fl, slew);
        bl = slewRate(rotPower, bl, slew);
        fr = slewRate(-rotPower, fr, slew);
        br = slewRate(-rotPower, br, slew);

        FL1.move(fl); 
        FL2.move(fl);
        BL1.move(bl); 
        BL2.move(bl);
        FR1.move(fr); 
        FR2.move(fr);
        BR1.move(br); 
        BR2.move(br);

        pros::delay(15);
    }

    StopBase();
}

// ---- X-drive PID with slew rate ----
void DriveToPoint_PID(double targetX, double targetY, double targetHeading, double maxSpeed, double slew) {

    double kP_xy = 3.0;
    double kI_xy = 0.0;
    double kD_xy = 0.3;

    double kP_rot = 3.0;
    double kI_rot = 0.0;
    double kD_rot = 0.2;

    double xIntegral = 0, yIntegral = 0, rIntegral = 0;
    double prevXError = 0, prevYError = 0, prevRotError = 0;

    double fl = 0, fr = 0, bl = 0, br = 0;

    while (true) {

        updateOdom();  // USE IMU-SYNCED ODOM

        double xError = targetX - xPos;
        double yError = targetY - yPos;
        double distance = sqrt(xError*xError + yError*yError);

        double rotError = targetHeading - (theta * 180.0 / M_PI);
        while (rotError > 180) rotError -= 360;
        while (rotError < -180) rotError += 360;

        if (distance < 0.6 && fabs(rotError) < 1.0) break;

        xIntegral += xError;
        yIntegral += yError;
        rIntegral += rotError;

        double xDerivative = xError - prevXError;
        double yDerivative = yError - prevYError;
        double rDerivative = rotError - prevRotError;

        prevXError = xError;
        prevYError = yError;
        prevRotError = rotError;

        double xPower = kP_xy * xError + kD_xy * xDerivative;
        double yPower = kP_xy * yError + kD_xy * yDerivative;
        double rotPower = kP_rot * rotError + kD_rot * rDerivative;

        xPower = KeepInRange(xPower, -maxSpeed, maxSpeed);
        yPower = KeepInRange(yPower, -maxSpeed, maxSpeed);
        rotPower = KeepInRange(rotPower, -maxSpeed, maxSpeed);

        double targetFL = yPower + xPower + rotPower;
        double targetFR = yPower - xPower - rotPower;
        double targetBL = yPower - xPower + rotPower;
        double targetBR = yPower + xPower - rotPower;

        fl = slewRate(targetFL, fl, slew);
        fr = slewRate(targetFR, fr, slew);
        bl = slewRate(targetBL, bl, slew);
        br = slewRate(targetBR, br, slew);

        FL1.move(fl); FL2.move(fl);
        BL1.move(bl); BL2.move(bl);
        FR1.move(fr); FR2.move(fr);
        BR1.move(br); BR2.move(br);

        pros::delay(15);
    }

    StopBase();
}