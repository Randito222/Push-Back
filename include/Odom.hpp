#pragma once

// Global pose (field frame)
// +X = right, +Y = forward, theta = radians
extern double odomX;       // inches
extern double odomY;       // inches
extern double odomTheta;   // radians

// Reset pose and sensors
void odomReset(double xIn = 0.0, double yIn = 0.0);

// Odom tracking task (run in a pros::Task)
void odomTask();