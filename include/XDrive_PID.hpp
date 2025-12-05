#pragma once
class RPID {
public:
    double kP, kI, kD;
    double integral = 0;
    double prevError = 0;
    double output = 0;

    // Optional: integral cap to prevent wind-up
    double integralLimit = 1000;

    RPID(double p, double i, double d) : kP(p), kI(i), kD(d) {}

    double calculate(double error) {
        // Prevent integral wind-up
        integral += error;
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;

        double derivative = error - prevError;
        prevError = error;

        output = (kP * error) + (kI * integral) + (kD * derivative);
        return output;
    }

    double calculateWithTarget(double target, double current) {
        double error = target - current;
        return calculate(error);
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }

    void setGains(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
};


double ticksToInches(double ticks);
double KeepInRange(float Value, double MinValue, double MaxValue);
void StopBase();
void updateOdometry();
void TurnToAngle_PID(double targetAngle, double maxSpeed, double exitError);
void DriveToPoint_PID(double targetX, double targetY, double targetHeading, double maxSpeed , double slew);