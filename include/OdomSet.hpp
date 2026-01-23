#pragma once
#include "pros/rtos.hpp"

// Global odom pose (field coordinates)
// Convention used by your DriveToPoint_OdomPID:
//  - odomX: +right (inches)
//  - odomY: +forward/upfield (inches)
//  - odomTheta: radians, 0 means facing +Y, CCW positive
extern double odomX;
extern double odomY;
extern double odomTheta;

// Odom functions
void updateOdom();
void resetOdom(); // resets to (0,0,0)
void printOdom();

// NEW: reset to a specific pose
void resetOdomPose(double x_in, double y_in, double thetaDeg);
void resetOdomPoseRad(double x_in, double y_in, double thetaRad);

// Task entry
void odomTask(void* ignore);
