// XDrive_PID.cpp
#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// From OdomSet.cpp
extern double odomX;
extern double odomY;
extern double odomTheta;  // radians

void initOdom();
void updateOdom();

// ---- Motor aliases (from subsystems.hpp) ----
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

// ---------- HELPERS ----------
double KeepInRange(double value, double minValue, double maxValue) {
  if (value < minValue) value = minValue;
  if (value > maxValue) value = maxValue;
  return value;
}

const double WHEEL_DIAMETER = 3.25;
const double TICKS_PER_REV = 180;   // depends on cartridge

double inchesToTicks(double inches) {
    return (inches / (WHEEL_DIAMETER * M_PI)) * TICKS_PER_REV;
}

double getStrafeTicks() {
    return (FL1.get_position() - FR1.get_position() - 
    BL1.get_position() + BR1.get_position()) / 4.0;
}

void resetDriveEncoders() {
    FL1.tare_position(); FL2.tare_position();
    BL1.tare_position(); BL2.tare_position();
    FR1.tare_position(); FR2.tare_position();
    BR1.tare_position(); BR2.tare_position();
}

// Get average of all drive encoders
double getDriveAvg() {
    return (FL1.get_position() + FL2.get_position() + BL1.get_position() + BL2.get_position() +
            FR1.get_position() + FR2.get_position() + BR1.get_position() + BR2.get_position()) / 8.0;
}

// Slew rate limiter (units per loop)
double slewRate(double target, double current, double rate) {
  if (current < target) current += rate;
  else if (current > target) current -= rate;

  if (fabs(target - current) < rate) current = target;
  return current;
}

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

// ---------- TURN PID (ABSOLUTE HEADING, NO IMU RESET) ----------
void TurnToAngle_PID(double targetAngleDeg,
                     double maxSpeed = 90.0,
                     double exitErrorDeg = 1.0,
                     double slew = 2.0) {
  // targetAngleDeg is a FIELD-centered absolute angle (0–360)
  double kP = 2.0;
  double kI = 0.0;
  double kD = 0.35;

  double integral = 0.0;
  double prevError = 0.0;

  double fl = 0.0, fr = 0.0, bl = 0.0, br = 0.0;
  int withinTol = 0;

  while (true) {
    double current = IMU.get_rotation();  // degrees
    double error = targetAngleDeg - current;

    // Normalize to [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    if (fabs(error) < exitErrorDeg) {
      withinTol++;
    } else {
      withinTol = 0;
    }
    if (withinTol > 8) break;  // hold in band for ~120ms

    integral += error;
    double derivative = error - prevError;
    prevError = error;

    double rotPower = kP * error + kI * integral + kD * derivative;
    rotPower = KeepInRange(rotPower, -maxSpeed, maxSpeed);

    // Slew
    fl = slewRate(rotPower,  fl, slew);
    bl = slewRate(rotPower,  bl, slew);
    fr = slewRate(-rotPower, fr, slew);
    br = slewRate(-rotPower, br, slew);

    FL1.move(fl); FL2.move(fl);
    BL1.move(bl); BL2.move(bl);
    FR1.move(fr); FR2.move(fr);
    BR1.move(br); BR2.move(br);

    pros::delay(15);
  }

  StopBase();
}

// ---------- DRIVE TO POINT (X-DRIVE, ROBOT-FRAME PID, HEADING LOCK) ----------
void DriveToPoint_PID(double targetX,
                      double targetY,
                      double targetHeadingDeg,
                      double maxSpeed,
                      double slew) {

  // PID gains – start here, then tune:
  double kP_fwd = 6.0;
  double kD_fwd = 0.6;

  double kP_strafe = 6.0;
  double kD_strafe = 0.6;

  double kP_rot = 3.0;
  double kD_rot = 0.3;

  double prevFwdErr = 0.0;
  double prevStrafeErr = 0.0;
  double prevRotErr = 0.0;

  double fl = 0.0, fr = 0.0, bl = 0.0, br = 0.0;
  int withinTol = 0;

  while (true) {
    updateOdom();  // updates odomX, odomY, odomTheta (radians)

    // Field-frame error
    double dx = targetX - odomX;
    double dy = targetY - odomY;
    double distance = std::sqrt(dx*dx + dy*dy);

    // Robot heading in degrees
    double headingDeg = IMU.get_rotation();
    double rotErrDeg = targetHeadingDeg - headingDeg;
    while (rotErrDeg > 180) rotErrDeg -= 360;
    while (rotErrDeg < -180) rotErrDeg += 360;

    // Exit condition with settle
    const double posTol = 0.75;   // inches
    const double angTol = 2.0;    // degrees

    if (distance < posTol && fabs(rotErrDeg) < angTol) {
      withinTol++;
    } else {
      withinTol = 0;
    }
    if (withinTol > 10) break;    // ~150ms inside the window

    // Transform field error into ROBOT frame
    double cosT = cos(odomTheta);
    double sinT = sin(odomTheta);

    // forward = along robot's +Y, strafe = along +X (left)
    double fwdErr    =  dx * sinT + dy * cosT;
    double strafeErr =  dx * cosT - dy * sinT;

    // Derivatives
    double dFwd    = fwdErr    - prevFwdErr;
    double dStrafe = strafeErr - prevStrafeErr;
    double dRot    = rotErrDeg - prevRotErr;

    prevFwdErr    = fwdErr;
    prevStrafeErr = strafeErr;
    prevRotErr    = rotErrDeg;

    // PID outputs
    double fwdPower    = kP_fwd    * fwdErr    + kD_fwd    * dFwd;
    double strafePower = kP_strafe * strafeErr + kD_strafe * dStrafe;
    double rotPower    = kP_rot    * rotErrDeg + kD_rot    * dRot;

    // Distance-based slowdown
    double slow = std::clamp(distance / 12.0, 0.25, 1.0);
    fwdPower    *= slow;
    strafePower *= slow;

    // Clamp
    fwdPower    = KeepInRange(fwdPower,    -maxSpeed, maxSpeed);
    strafePower = KeepInRange(strafePower, -maxSpeed, maxSpeed);
    rotPower    = KeepInRange(rotPower,    -maxSpeed, maxSpeed);

    // X-drive mixing (y = forward, x = strafe)
    double targetFL = fwdPower + strafePower + rotPower;
    double targetFR = fwdPower - strafePower - rotPower;
    double targetBL = fwdPower - strafePower + rotPower;
    double targetBR = fwdPower + strafePower - rotPower;

    // Slew each wheel
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

void PID_Movement(double inches, double maxPower) {

    double targetTicks = inchesToTicks(inches);

    double kP = 0.9;
    double kI = 0.004;
    double kD = 2.5;

    double integral = 0;
    double lastError = 0;

    resetDriveEncoders();

    while (true) {

        double current = getDriveAvg();
        double error = targetTicks - current;

        if (fabs(error) < 5) break;   // within 5 ticks → done

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = KeepInRange(power, -maxPower, maxPower);

        // Move straight forward
        FL1.move(power); FL2.move(power);
        BL1.move(power); BL2.move(power);
        FR1.move(power); FR2.move(power);
        BR1.move(power); BR2.move(power);

        pros::delay(10);
    }

    // Stop robot
    StopBase();
}

void PID_Strafe(double inches, double maxPower) {

    double targetTicks = inchesToTicks(inches);

    double kP = 0.6;
    double kI = 0.0;
    double kD = 0.1;

    double integral = 0;
    double lastError = 0;

    resetDriveEncoders();

    while (true) {

        double current = getStrafeTicks();
        double error = targetTicks - current;

        if (fabs(error) < 5) break;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = KeepInRange(power, -maxPower, maxPower);

        // X-drive strafe wheel mixing:
        FL1.move( power); FL2.move( power);
        BL1.move(-power); BL2.move(-power);
        FR1.move(-power); FR2.move(-power);
        BR1.move( power); BR2.move( power);

        pros::delay(10);
    }

    // Stop
    StopBase();
}

void PID_Turn(double targetDeg, double maxPower) {

    double kP = 2.0;
    double kD = 0.3;

    double lastError = 0;

    while (true) {

        double heading = IMU.get_rotation();
        double error = targetDeg - heading;

        // normalize
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        if (fabs(error) < 1.0) break;

        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kD * derivative;
        power = KeepInRange(power, -maxPower, maxPower);

        FL1.move( power); FL2.move( power);
        BL1.move( power); BL2.move( power);
        FR1.move(-power); FR2.move(-power);
        BR1.move(-power); BR2.move(-power);

        pros::delay(10);
    }

    // Stop
    StopBase();
}