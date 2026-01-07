#include "XDrive_PID.hpp"
#include "OdomSet.hpp"
#include "subsystems.hpp"
#include <cmath>

// Motor aliases
#define FL1 Front_Left_1
#define FL2 Front_Left_2
#define FR1 Front_Right_1
#define FR2 Front_Right_2
#define BL1 Back_Left_1
#define BL2 Back_Left_2
#define BR1 Back_Right_1
#define BR2 Back_Right_2

static double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static double Myslew(double target, double current, double step) {
    double diff = target - current;
    if (std::fabs(diff) <= step) return target;
    return current + (diff > 0 ? step : -step);
}

static double wrapDeg(double deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

static void stopDrive() {
    FL1.move(0); FL2.move(0);
    FR1.move(0); FR2.move(0);
    BL1.move(0); BL2.move(0);
    BR1.move(0); BR2.move(0);
}

void DriveToPoint_OdomPID(
    double targetX,
    double targetY,
    double targetHeadingDeg,
    int    maxVolt,
    int    timeout_ms,
    double slewRateV
) {
    // =============================
    // PID controllers (starter values)
    // =============================
    RPID xPID(0.4, 0.0, 0.3);
    RPID yPID(0.4, 0.0, 0.3);
    RPID tPID(0.2,  0.0, 0.9);   // turn during translation (safer)
    tPID.integralLimit = 300;   // usually keep turn I small (even if I=0)

    double fl = 0, fr = 0, bl = 0, br = 0;
    int settled = 0;
    int start = pros::millis();

    while (pros::millis() - start < timeout_ms) {

        // =============================
        // FIELD-CENTRIC ERROR
        // =============================
        double xErr = targetX - odomX;
        double yErr = targetY - odomY;

        double headingDeg = odomTheta * 180.0 / M_PI;
        double tErr = wrapDeg(targetHeadingDeg - headingDeg);

        // deadband prevents spiral “micro-corrections”
        if (std::fabs(tErr) < 1.0) tErr = 0;

        // =============================
        // SETTLE CHECK
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
        // FIELD -> ROBOT TRANSFORM
        // =============================
        double sinH = std::sin(odomTheta);
        double cosH = std::cos(odomTheta);

        double robotX =  xErr * cosH + yErr * sinH;   // strafe
        double robotY = -xErr * sinH + yErr * cosH;   // forward

        // =============================
        // PID OUTPUTS (robot frame)
        // =============================
        double xOut = clamp(xPID.calculate(robotX), -maxVolt, maxVolt);
        double yOut = clamp(yPID.calculate(robotY), -maxVolt, maxVolt);

        // Scale turn down while translating (prevents looping)
        double driveMag = std::hypot(robotX, robotY);
        double turnScale = clamp(1.0 - driveMag / 24.0, 0.3, 1.0);

        double tOut = clamp(tPID.calculate(tErr) * turnScale, -maxVolt, maxVolt);

        // =============================
        // X-DRIVE MIXING
        // =============================
        double tFL = yOut + xOut + tOut;
        double tFR = yOut - xOut - tOut;
        double tBL = yOut - xOut + tOut;
        double tBR = yOut + xOut - tOut;

        // =============================
        // SLEW
        // =============================
        fl = Myslew(tFL, fl, slewRateV);
        fr = Myslew(tFR, fr, slewRateV);
        bl = Myslew(tBL, bl, slewRateV);
        br = Myslew(tBR, br, slewRateV);

        // =============================
        // APPLY
        // =============================
        FL1.move(fl); FL2.move(fl);
        FR1.move(fr); FR2.move(fr);
        BL1.move(bl); BL2.move(bl);
        BR1.move(br); BR2.move(br);

        pros::delay(10);
    }

    stopDrive();
}
