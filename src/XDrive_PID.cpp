#include "XDrive_PID.hpp"
#include "main.h"
#include "subsystems.hpp"
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



// =============================
// Constants
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

    stopDrive();
}

void DriveToPoint_OdomPID(
    double targetX,
    double targetY,
    double targetHeading,
    int    maxSpeed,
    int    timeout_ms,
    double slewRateV
) {
    // =============================
    // PID tuning 
    // =============================
    PIDTest xPID {9.0, 0.0, 40.0};
    PIDTest yPID {9.0, 0.0, 40.0};
    PIDTest turnPID {3.0, 0.0, 24.0}; // heading PID (degrees)

    // reset PIDs for consistent repeated calls
    xPID.reset();
    yPID.reset();
    turnPID.reset();

    // minimum output to overcome static friction
    // (These are motor "move" units: -127..127)
    const double MIN_XY   = 8.0;   
    const double MIN_TURN = 6.0;  

    auto applyMin = [&](double v, double minv) -> double {
        if (std::fabs(v) < 1e-6) return 0.0;
        if (std::fabs(v) < minv) return (v > 0) ? minv : -minv;
        return v;
    };

    double fl = 0, fr = 0, bl = 0, br = 0;
    int settled = 0;
    int start = pros::millis();

    while (pros::millis() - start < timeout_ms) {

        // =============================
        // ODOM ERRORS (FIELD-CENTRIC)
        // =============================
        double xErr = targetX - odomX;
        double yErr = targetY - odomY;

        // heading in degrees
        double headingDeg = odomTheta * 180.0 / M_PI;
        double tErr = targetHeading - headingDeg;

        // wrap heading error to [-180, 180]
        while (tErr > 180) tErr -= 360;
        while (tErr < -180) tErr += 360;

        // deadband to stop tiny turn jitter
        if (std::fabs(tErr) < 1.0) tErr = 0;

        // =============================
        // SETTLING CHECK
        // =============================
        if (std::fabs(xErr) < 0.5 &&
            std::fabs(yErr) < 0.5 &&
            std::fabs(tErr) < 1.0)
        {
            settled += 10;
            if (settled > 200) break;
        } else {
            settled = 0;
        }

        // =============================
        // FIELD → ROBOT TRANSFORM
        // =============================
        double sinH = std::sin(odomTheta);
        double cosH = std::cos(odomTheta);

        double robotX =  xErr * cosH + yErr * sinH;
        double robotY = -xErr * sinH + yErr * cosH;

        // =============================
        // PID OUTPUTS
        // =============================
        double xOut = clamp(xPID.step(robotX), -maxSpeed, maxSpeed);
        double yOut = clamp(yPID.step(robotY), -maxSpeed, maxSpeed);

        // scale turn while translating to prevent spiraling 
        double driveMag = std::hypot(robotX, robotY);
        double turnScale = clamp(1.0 - (driveMag / 24.0), 0.3, 1.0);

        double tOut = clamp(turnPID.step(tErr) * turnScale, -maxSpeed, maxSpeed);

        // apply minimums (only when nonzero) 
        xOut = applyMin(xOut, MIN_XY);
        yOut = applyMin(yOut, MIN_XY);
        tOut = applyMin(tOut, MIN_TURN);

        // =============================
        // X-DRIVE MIXING
        // =============================
        double tFL = yOut + xOut + tOut;
        double tFR = yOut - xOut - tOut;
        double tBL = yOut - xOut + tOut;
        double tBR = yOut + xOut - tOut;

        double maxMag = std::max({
            std::fabs(tFL),
            std::fabs(tFR),
            std::fabs(tBL),
            std::fabs(tBR)
        });

        if (maxMag > maxSpeed) {
            double scale = (double)maxSpeed / maxMag;
            tFL *= scale;
            tFR *= scale;
            tBL *= scale;
            tBR *= scale;
        }

        // =============================
        // SLEW RATE
        // =============================
        fl = Myslew(tFL, fl, slewRateV);
        fr = Myslew(tFR, fr, slewRateV);
        bl = Myslew(tBL, bl, slewRateV);
        br = Myslew(tBR, br, slewRateV);

        // =============================
        // APPLY POWER
        // =============================
        FL1.move((int)fl); FL2.move((int)fl);
        FR1.move((int)fr); FR2.move((int)fr);
        BL1.move((int)bl); BL2.move((int)bl);
        BR1.move((int)br); BR2.move((int)br);

        pros::delay(10);
    }

    stopDrive();
}