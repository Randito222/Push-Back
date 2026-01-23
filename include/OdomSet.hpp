#pragma once
#include "pros/rtos.hpp"

// Global odom pose (field coordinates)
extern double odomX;      // inches (right +)
extern double odomY;      // inches (forward +)
extern double odomTheta;  // radians (CCW +)

// Odom functions
void updateOdom();
void resetOdom();
void printOdom();

// Task entry (PROS Task expects void(*)(void*))
void odomTask(void* ignore);