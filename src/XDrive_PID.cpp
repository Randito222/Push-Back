#include "XDrive_PID.hpp"
#include "OdomSet.hpp"
#include "subsystems.hpp"
#include "main.h"

#include <cmath>

// =============================
// Motor aliases (local only)
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
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// =============================
// Utility Helpers (from header)
// =============================
double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double slewRate(double target, double current, double maxDelta) {
    double diff = target - current;
    if (std::fabs(diff) <= maxDelta) return target;
    return current + (diff > 0 ? maxDelta : -maxDelta);
}

void StopBase() {
    FL1.move(0); FL2.move(0);
    FR1.move(0); FR2.move(0);
    BL1.move(0); BL2.move(0);
    BR1.move(0); BR2.move(0);
}

// =============================
// Internal PID helpers (cpp-only)
// =============================
struct PIDTest {
    double kP, kI, kD;
    double integral = 0;
    double prev = 0;

    double step(double err) {
        integral += err;
        double out = kP * err + kI * integral + kD * (err - prev);
        prev = err;
        return out;
    }
};

static double wrapRad(double a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

// =============================
// X-Drive Odometry PID
// =============================
void DriveToPoint_PID(
    double targetX,
    double targetY,
    double targetHeadingDeg,
    double maxSpeed,
    double slew
) {
    // --- PID tuning ---
    PIDTest drivePID {4.2, 0.002, 0.3};
    PIDTest turnPID  {2.0, 0.0,   0.4};

    const double targetHeading = targetHeadingDeg * DEG2RAD;

    double fl = 0, fr = 0, bl = 0, br = 0;
    int settleTime = 0;

    while (true) {
        updateOdom();

        // -----------------------------
        // World-frame error
        // -----------------------------
        double dx = targetX - odomX;
        double dy = targetY - odomY;

        double rotErr = wrapRad(targetHeading - odomTheta);

        // -----------------------------
        // Convert to robot frame
        // -----------------------------
        double cosT = std::cos(-odomTheta);
        double sinT = std::sin(-odomTheta);

        double fwdErr    = dy * cosT - dx * sinT;
        double strafeErr = dy * sinT + dx * cosT;

        // -----------------------------
        // Settling check
        // -----------------------------
        if (std::fabs(fwdErr) < 0.5 &&
            std::fabs(strafeErr) < 0.5 &&
            std::fabs(rotErr * RAD2DEG) < 1.0)
        {
            settleTime += 15;
            if (settleTime > 200) break;
        } else {
            settleTime = 0;
        }

        // -----------------------------
        // PID outputs
        // -----------------------------
        double fwdPower    = drivePID.step(fwdErr);
        double strafePower = drivePID.step(strafeErr);
        double rotPower    = turnPID.step(rotErr);

        fwdPower    = clamp(fwdPower,    -maxSpeed, maxSpeed);
        strafePower = clamp(strafePower, -maxSpeed, maxSpeed);
        rotPower    = clamp(rotPower,    -maxSpeed, maxSpeed);

        // -----------------------------
        // X-drive mixing
        // -----------------------------
        double s = -strafePower; // keep your strafe fix

        double tFL = fwdPower + s + rotPower;
        double tFR = fwdPower - s - rotPower;
        double tBL = fwdPower - s + rotPower;
        double tBR = fwdPower + s - rotPower;

        // -----------------------------
        // Slew rate limiting
        // -----------------------------
        fl = slewRate(tFL, fl, slew);
        fr = slewRate(tFR, fr, slew);
        bl = slewRate(tBL, bl, slew);
        br = slewRate(tBR, br, slew);

        // -----------------------------
        // Apply to motors
        // -----------------------------
        FL1.move(fl); FL2.move(fl);
        FR1.move(fr); FR2.move(fr);
        BL1.move(bl); BL2.move(bl);
        BR1.move(br); BR2.move(br);

        pros::delay(15);
    }

    StopBase();
}
