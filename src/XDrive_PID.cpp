#include "main.h"
#include "subsystems.hpp"

RPID xPID(0.5, 0.0, 0.1);   // Strafe
RPID yPID(0.5, 0.0, 0.1);   // Forward/back
RPID rotPID(0.5, 0.0, 0.1); // Turn

double distanceError = 0.0; // Global so it can be accessed by this function

void pid_wait_until_distance(double threshold) {
  while (true) {
    if (fabs(distanceError) <= threshold) break;
    pros::delay(10);
  }
}

void x_drive_pid_task(double targetX, double targetY, double targetTheta) {
  while (true) {
    // Read odometry pose (in inches and degrees)
    double currentX = chassis.odom_x_get();
    double currentY = chassis.odom_y_get();
    double currentTheta = chassis.odom_theta_get();

     // Calculate error
    double xError = targetX - currentX;
    double yError = targetY - currentY;

    // Update distance error for external usage
    distanceError = sqrt((xError * xError) + (yError * yError));

    // Calculate PID outputs 
    double xPower = xPID.calculateWithTarget(targetX, currentX);
    double yPower = yPID.calculateWithTarget(targetY, currentY);
    

    // Normalize angle to range [-180, 180]
    double thetaError = targetTheta - currentTheta;
    while (thetaError > 180) thetaError -= 360;
    while (thetaError < -180) thetaError += 360;

    double rotPower = rotPID.calculate(thetaError);

    // Holonomic drive motor mixing
    double fl = yPower + xPower + rotPower;
    double fr = yPower - xPower - rotPower;
    double bl = yPower - xPower + rotPower;
    double br = yPower + xPower - rotPower;

    // Clamp motor powers to [-127, 127] if needed
    fl = std::clamp(fl, -127.0, 127.0);
    fr = std::clamp(fr, -127.0, 127.0);
    bl = std::clamp(bl, -127.0, 127.0);
    br = std::clamp(br, -127.0, 127.0);

    // Set motor powers
    FrontLeft.move(fl);
    FrontRight.move(fr);
    BackLeft.move(bl);
    BackRight.move(br);

    pros::delay(10);
  }


  FrontLeft.move(0);
  FrontRight.move(0);
  BackLeft.move(0);
  BackRight.move(0);
}



