#include "XDrive_PID.hpp"
#include "main.h"
<<<<<<< HEAD
=======
#include "subsystems.hpp"
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
#include <cmath>

// =============================
// Motor aliases
// =============================
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

<<<<<<< HEAD
// =============================
// IMU
// =============================
extern pros::Imu imu;

// =============================
// Constants (TUNE)
=======


// =============================
// Constants
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
// =============================
constexpr double WHEEL_DIAM_IN = 3.25;
constexpr double GEAR_RATIO   = 1.0;
constexpr double PI = 3.141592653589793;

// =============================
// Utility
// =============================
double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

double Myslew(double target, double current, double step) {
    double diff = target - current;
    if (std::fabs(diff) <= step) return target;
    return current + (diff > 0 ? step : -step);
}

// =============================
// PID
// =============================
struct PIDTest {
<<<<<<< HEAD
    double kP, kI, kD;
    double integral = 0;
    double prevErr  = 0;
    double iLimit   = 50;

    double step(double err) {
        integral += err;
        integral = clamp(integral, -iLimit, iLimit);
        double out = kP * err + kI * integral + kD * (err - prevErr);
        prevErr = err;
        return out;
    }

    void reset() {
        integral = 0;
        prevErr  = 0;
    }
};

// =============================
// Encoder helpers
// =============================
static double degToIn(double deg) {
    return (deg / 360.0) * PI * WHEEL_DIAM_IN / GEAR_RATIO;
}

static double avg(double a, double b) { return (a + b) * 0.5; }

static double flDeg() { return avg(FL1.get_position(), FL2.get_position()); }
static double frDeg() { return avg(FR1.get_position(), FR2.get_position()); }
static double blDeg() { return avg(BL1.get_position(), BL2.get_position()); }
static double brDeg() { return avg(BR1.get_position(), BR2.get_position()); }

// =============================
// Pseudo-odometry (encoder only)
// =============================
static double xPos() {
    return degToIn((flDeg() - frDeg() - blDeg() + brDeg()) / 4.0);
}

static double yPos() {
    return degToIn((flDeg() + frDeg() + blDeg() + brDeg()) / 4.0);
}

// =============================
// IMU heading helpers
// =============================
static double wrapDeg(double deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

static double imuHeading() {
    return wrapDeg(imu.get_heading());
}

// =============================
// Stop
// =============================
void stopDrive() {
    FL1.move_voltage(0); FL2.move_voltage(0);
    FR1.move_voltage(0); FR2.move_voltage(0);
    BL1.move_voltage(0); BL2.move_voltage(0);
    BR1.move_voltage(0); BR2.move_voltage(0);
}

// =============================
// MAIN PID
// =============================
void DriveToPoint_PID(
    double targetX,
    double targetY,
    double targetHeading,
    int    maxVolt,
    int    timeout_ms,
    double slewRateV
) {
    // Reset encoders
    FL1.tare_position(); FL2.tare_position();
    FR1.tare_position(); FR2.tare_position();
    BL1.tare_position(); BL2.tare_position();
    BR1.tare_position(); BR2.tare_position();

    // PID tuning (starter values)
    PIDTest xPID    {900, 0.0, 350};
    PIDTest yPID    {900, 0.0, 350};
    PIDTest turnPID {80,  0.0, 500};  // IMU turn PID

    double fl = 0, fr = 0, bl = 0, br = 0;
    int settled = 0;
    int start = pros::millis();

    while (pros::millis() - start < timeout_ms) {
        // Errors
        double xErr = targetX - xPos();
        double yErr = targetY - yPos();
        double tErr = wrapDeg(targetHeading - imuHeading());

        // Settling check
        if (std::fabs(xErr) < 0.5 &&
            std::fabs(yErr) < 0.5 &&
            std::fabs(tErr) < 1.0)
        {
            settled += 10;
            if (settled > 200) break;
        } else settled = 0;

        // PID outputs
        double xOut = clamp(xPID.step(xErr), -maxVolt, maxVolt);
        double yOut = clamp(yPID.step(yErr), -maxVolt, maxVolt);
        double tOut = clamp(turnPID.step(tErr), -maxVolt, maxVolt);

        // X-drive mixing
        double tFL = yOut + xOut + tOut;
        double tFR = yOut - xOut - tOut;
        double tBL = yOut - xOut + tOut;
        double tBR = yOut + xOut - tOut;

        // Slew rate
        fl = Myslew(tFL, fl, slewRateV);
        fr = Myslew(tFR, fr, slewRateV);
        bl = Myslew(tBL, bl, slewRateV);
        br = Myslew(tBR, br, slewRateV);

        // Apply
        FL1.move_voltage(fl); FL2.move_voltage(fl);
        FR1.move_voltage(fr); FR2.move_voltage(fr);
        BL1.move_voltage(bl); BL2.move_voltage(bl);
        BR1.move_voltage(br); BR2.move_voltage(br);

        pros::delay(10);
    }

=======
  double kP=0, kI=0, kD=0;
  double integral=0, prevErr=0;
  double iLimit=50;
  double iZone=5;   // only integrate when |err| < iZone

  double step(double err) {
    if (fabs(err) < iZone) integral += err;
    else integral = 0;

    integral = clamp(integral, -iLimit, iLimit);

    double deriv = err - prevErr;
    prevErr = err;

    return kP*err + kI*integral + kD*deriv;
  }

  void reset() { integral=0; prevErr=0; }
};


// =============================
// Encoder helpers
// =============================
static double degToIn(double deg) {
    return (deg / 360.0) * PI * WHEEL_DIAM_IN / GEAR_RATIO;
}

static double avg(double a, double b) { return (a + b) * 0.5; }

static double flDeg() { return avg(FL1.get_position(), FL2.get_position()); }
static double frDeg() { return avg(FR1.get_position(), FR2.get_position()); }
static double blDeg() { return avg(BL1.get_position(), BL2.get_position()); }
static double brDeg() { return avg(BR1.get_position(), BR2.get_position()); }

// =============================
// Pseudo-odometry (encoder only)
// =============================
static double xPos() {
    return degToIn((flDeg() - frDeg() - blDeg() + brDeg()) / 4.0);
}

static double yPos() {
    return degToIn((flDeg() + frDeg() + blDeg() + brDeg()) / 4.0);
}

// =============================
// IMU heading helpers
// =============================
static double wrapDeg(double deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

static double imuHeading() {
    return wrapDeg(IMU.get_rotation());
}

// =============================
// Stop
// =============================
void stopDrive() {
    FL1.move(0); FL2.move(0);
    FR1.move(0); FR2.move(0);
    BL1.move(0); BL2.move(0);
    BR1.move(0); BR2.move(0);
}

// =============================
// MAIN PID
// =============================
void DriveToPoint_PID(
    double targetX,
    double targetY,
    double targetHeading,
    int    maxSpeed,
    int    timeout_ms,
    double slewRateV
) {
    // Reset encoders
    FL1.tare_position(); FL2.tare_position();
    FR1.tare_position(); FR2.tare_position();
    BL1.tare_position(); BL2.tare_position();
    BR1.tare_position(); BR2.tare_position();

    // PID tuning (starter values)
    PIDTest xPID {10.0, 0.02, 40.0};
    PIDTest yPID {10.0, 0.02, 40.0};
    PIDTest turnPID {3.0, 0.01, 24.0};

    double fl = 0, fr = 0, bl = 0, br = 0;
    int settled = 0;
    int start = pros::millis();

    while (pros::millis() - start < timeout_ms) {
        // Errors
        double xErr = targetX - xPos();
        double yErr = targetY - yPos();
        double tErr = wrapDeg(targetHeading - imuHeading());

        // Settling check
        if (std::fabs(xErr) < 0.5 &&
            std::fabs(yErr) < 0.5 &&
            std::fabs(tErr) < 1.0)
        {
            settled += 10;
            if (settled > 200) break;
        } else settled = 0;

        // PID outputs
        double xOut = clamp(xPID.step(xErr), -maxSpeed, maxSpeed);
        double yOut = clamp(yPID.step(yErr), -maxSpeed, maxSpeed);
        double tOut = clamp(turnPID.step(tErr), -maxSpeed, maxSpeed);

        // X-drive mixing
        double tFL = yOut + xOut + tOut;
        double tFR = yOut - xOut - tOut;
        double tBL = yOut - xOut + tOut;
        double tBR = yOut + xOut - tOut;

        // Slew rate
        fl = Myslew(tFL, fl, slewRateV);
        fr = Myslew(tFR, fr, slewRateV);
        bl = Myslew(tBL, bl, slewRateV);
        br = Myslew(tBR, br, slewRateV);

        // Apply
        FL1.move(fl); FL2.move(fl);
        FR1.move(fr); FR2.move(fr);
        BL1.move(bl); BL2.move(bl);
        BR1.move(br); BR2.move(br);

        pros::delay(10);
    }

>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
    stopDrive();
}

void DriveToPoint_OdomPID(
    double targetX,
    double targetY,
<<<<<<< HEAD
    double targetHeading,
    int    maxVolt,
    int    timeout_ms,
    double slewRateV
) {
    // =============================
    // PID tuning (starter values)
    // =============================
    PIDTest xPID    {900, 0.0, 350};
    PIDTest yPID    {900, 0.0, 350};
    PIDTest turnPID {80,  0.0, 500};   // heading PID (degrees)

=======
    double targetHeadingDeg,
    int    maxSpeed,     // move() units: 0..127
    int    timeout_ms,
    double slewRateV     // move() units per loop, ex: 4..15
) {
    // =============================
    // PID tuning 
    // =============================
    // These are for move() output (-127..127) and inches/deg errors.
    PIDTest xPID    {10.0, 0.008, 30.0};  // strafe control
    PIDTest yPID    {10.0, 0.008, 30.0};  // forward control
    PIDTest turnPID { 3.0, 0.00, 18.0};  // heading control (degrees)

    xPID.reset();
    yPID.reset();
    turnPID.reset();

    // =============================
    // Minimum output (break friction)
    // =============================
    // Only applied when we're far enough from target (error-gated).
    const double MIN_XY   = 8.0;   // 6..12 typical
    const double MIN_TURN = 6.0;   // 4..10 typical

    // Don't force minimum output once we're basically there:
    const double XY_ERR_GATE   = 0.6;  // inches
    const double TURN_ERR_GATE = 2.0;  // degrees

    auto applyMin = [&](double out, double minv) -> double {
        if (std::fabs(out) < 1e-6) return 0.0;
        if (std::fabs(out) < minv) return (out > 0) ? minv : -minv;
        return out;
    };

    auto applyMinWithErrGate = [&](double out, double err, double minv, double gate) -> double {
        if (std::fabs(err) < gate) return 0.0;   // stop jitter near target
        return applyMin(out, minv);
    };

    // =============================
    // Loop state
    // =============================
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
    double fl = 0, fr = 0, bl = 0, br = 0;
    int settled = 0;
    int start = pros::millis();

    while (pros::millis() - start < timeout_ms) {

        // =============================
<<<<<<< HEAD
        // ODOM ERRORS (FIELD-CENTRIC)
=======
        // FIELD ERRORS (inches)
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
        // =============================
        double xErr = targetX - odomX;
        double yErr = targetY - odomY;

<<<<<<< HEAD
        // heading in degrees
        double headingDeg = odomTheta * 180.0 / M_PI;
        double tErr = targetHeading - headingDeg;

        // wrap heading error to [-180, 180]
        while (tErr > 180) tErr -= 360;
        while (tErr < -180) tErr += 360;

        // =============================
        // SETTLING CHECK
        // =============================
        if (std::fabs(xErr) < 0.5 &&
            std::fabs(yErr) < 0.5 &&
            std::fabs(tErr) < 1.0)
        {
            settled += 10;
            if (settled > 200) break;
=======
        // heading error (degrees, wrapped)
        double headingDeg = odomTheta * 180.0 / M_PI;
        double tErr = targetHeadingDeg - headingDeg;
        while (tErr > 180) tErr -= 360;
        while (tErr < -180) tErr += 360;

        // optional tiny deadband to stop jitter
        if (std::fabs(tErr) < 1.0) tErr = 0;

        // =============================
        // SETTLE CHECK (distance based)
        // =============================
        double distErr = std::hypot(xErr, yErr);

        if (distErr < 0.5 && std::fabs(tErr) < 0.7) {
            settled += 10;
            if (settled > 200) break;  // ~200ms stable
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
        } else {
            settled = 0;
        }

        // =============================
<<<<<<< HEAD
        // FIELD → ROBOT TRANSFORM
=======
        // FIELD -> ROBOT transform
        // robotX = strafe error, robotY = forward error
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
        // =============================
        double sinH = std::sin(odomTheta);
        double cosH = std::cos(odomTheta);

<<<<<<< HEAD
        double robotX =  xErr * cosH + yErr * sinH;
        double robotY = -xErr * sinH + yErr * cosH;

        // =============================
        // PID OUTPUTS
        // =============================
        double xOut = clamp(xPID.step(robotX), -maxVolt, maxVolt);
        double yOut = clamp(yPID.step(robotY), -maxVolt, maxVolt);
        double tOut = clamp(turnPID.step(tErr),  -maxVolt, maxVolt);

        // =============================
        // X-DRIVE MIXING
=======
        double robotX =  xErr * cosH + yErr * sinH;   // strafe
        double robotY = -xErr * sinH + yErr * cosH;   // forward

        // =============================
        // PID outputs (robot frame)
        // =============================
        double xOut = clamp(xPID.step(robotX), -maxSpeed, maxSpeed);
        double yOut = clamp(yPID.step(robotY), -maxSpeed, maxSpeed);

        // Scale turning down while translating to prevent spiraling
        // 24 inches is an adjustable “turn reduction distance”
        double turnScale = clamp(1.0 - (distErr / 24.0), 0.25, 1.0);
        double tOut = clamp(turnPID.step(tErr) * turnScale, -maxSpeed, maxSpeed);

        // =============================
        // Minimum output with gating (prevents end jitter)
        // =============================
        xOut = applyMinWithErrGate(xOut, robotX, MIN_XY, XY_ERR_GATE);
        yOut = applyMinWithErrGate(yOut, robotY, MIN_XY, XY_ERR_GATE);
        tOut = applyMinWithErrGate(tOut, tErr,   MIN_TURN, TURN_ERR_GATE);

        // =============================
        // X-DRIVE mixing
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
        // =============================
        double tFL = yOut + xOut + tOut;
        double tFR = yOut - xOut - tOut;
        double tBL = yOut - xOut + tOut;
        double tBR = yOut + xOut - tOut;

        // =============================
<<<<<<< HEAD
        // SLEW RATE
=======
        // Normalize so max wheel <= maxSpeed
        // =============================
        double maxMag = std::max({ std::fabs(tFL), std::fabs(tFR), std::fabs(tBL), std::fabs(tBR) });
        if (maxMag > maxSpeed) {
            double scale = (double)maxSpeed / maxMag;
            tFL *= scale; tFR *= scale; tBL *= scale; tBR *= scale;
        }

        // =============================
        // Slew rate limit
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
        // =============================
        fl = Myslew(tFL, fl, slewRateV);
        fr = Myslew(tFR, fr, slewRateV);
        bl = Myslew(tBL, bl, slewRateV);
        br = Myslew(tBR, br, slewRateV);

        // =============================
<<<<<<< HEAD
        // APPLY VOLTAGE
        // =============================
        FL1.move_voltage(fl); FL2.move_voltage(fl);
        FR1.move_voltage(fr); FR2.move_voltage(fr);
        BL1.move_voltage(bl); BL2.move_voltage(bl);
        BR1.move_voltage(br); BR2.move_voltage(br);
=======
        // Apply to motors (move: -127..127)
        // =============================
        FL1.move((int)fl); FL2.move((int)fl);
        FR1.move((int)fr); FR2.move((int)fr);
        BL1.move((int)bl); BL2.move((int)bl);
        BR1.move((int)br); BR2.move((int)br);

        // =============================
        // Debug
        // =============================
        pros::lcd::print(4, "FieldErr x%.1f y%.1f d%.1f", xErr, yErr, distErr);
        pros::lcd::print(5, "RobotErr rx%.1f ry%.1f", robotX, robotY);
        pros::lcd::print(6, "Head %.1f tErr %.1f", headingDeg, tErr);
        pros::lcd::print(7, "Out x%.1f y%.1f t%.1f", xOut, yOut, tOut);
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9

        pros::delay(10);
    }

    stopDrive();
}
