#pragma once
#include <cmath>

// ---- Odometry position variables ----
extern double &xPos;   // inches
extern double &yPos ;   // inches
extern double &theta;  // radians

void initOdom();
void updateOdom();
static double getHeadingRad();
void odomTask();