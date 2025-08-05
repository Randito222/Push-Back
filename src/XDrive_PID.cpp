#include "main.h"

PID xPID(0.5, 0.0, 0.1);   // Strafe
PID yPID(0.5, 0.0, 0.1);   // Forward/back
PID rotPID(0.5, 0.0, 0.1); // Turn

// Target position
double targetX = 24.0;     // 24 inches right
double targetY = 36.0;     // 36 inches forward
double targetTheta = 0.0;  // No rotation


void x_drive_pid_task(double TarX,double TarY, double TarTheta) {
  while (true) {
    // Get your robot's current pose from odometry
    double currentX = // Get x position in inches
    double currentY = /* your y position (inches) */;
    double currentTheta = /* your heading (degrees) */;

    // Calculate error
    double xError = targetX - currentX;
    double yError = targetY - currentY;
    double thetaError = targetTheta - currentTheta;

    // PID outputs
    double xPower = xPID.calculate(xError);
    double yPower = yPID.calculate(yError);
    double rotPower = rotPID.calculate(thetaError);

    // Convert to motor powers for holonomic drive
    double fl = yPower + xPower + rotPower;
    double fr = yPower - xPower - rotPower;
    double bl = yPower - xPower + rotPower;
    double br = yPower + xPower - rotPower;

    // Set motor powers (clamp if needed)
    motorFL.move(fl);
    motorFR.move(fr);
    motorBL.move(bl);
    motorBR.move(br);

    pros::delay(10);
  }
}

