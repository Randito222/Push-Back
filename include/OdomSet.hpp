#pragma once
#include <cmath>

// =============================
// Odometry State (GLOBAL)
// =============================
extern double odomX;      // inches
extern double odomY;      // inches
extern double odomTheta;  // radians

// =============================
// Odometry Functions
// =============================
void updateOdom();
void resetOdom();
void odomTask();
