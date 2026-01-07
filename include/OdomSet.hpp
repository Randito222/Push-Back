#pragma once


// Global odom pose (field coordinates)
extern double odomX;      // inches (right +)
extern double odomY;      // inches (forward +)
extern double odomTheta;  // radians (CCW +)

// Odom functions
void updateOdom();
void resetOdom(double x = 0.0, double y = 0.0, double headingDeg = 0.0);
void printOdom();
