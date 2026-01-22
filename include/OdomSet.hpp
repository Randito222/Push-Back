#pragma once
#include "pros/rtos.hpp"

<<<<<<< HEAD
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
void printOdom();
=======
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
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
