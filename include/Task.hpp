#pragma once
#include "pros/rtos.hpp"

// =============================
// Color Sorting
// =============================
double Get_Color1();
double Get_Color2();

void Color_Mode();
void Color_Sorter();

// =============================
// Drive Mode
// =============================
void Drive_Controls_swap();

// =============================
// Odometry Task
// =============================
void odomTask();

// =============================
// Global Task State
// =============================
extern int colorMode;           // 0 = red, 1 = blue
extern int BackIntakeControl;

// =============================
// Tasks
// =============================
extern pros::Task Color_Mode_Task;
extern pros::Task Color_Sorter_Task;
extern pros::Task Drive_Controls_task;
// extern pros::Task odom_task;
#pragma once
#include "pros/rtos.hpp"

// =============================
// Color Sorting
// =============================
double Get_Color1();
double Get_Color2();

void Color_Mode();
void Color_Sorter();

// =============================
// Drive Mode
// =============================
void Drive_Controls_swap();

// =============================
// Odometry Task
// =============================
void odomTask();

// =============================
// Global Task State
// =============================
extern int colorMode;           // 0 = red, 1 = blue
extern int BackIntakeControl;

// =============================
// Tasks
// =============================
extern pros::Task Color_Mode_Task;
extern pros::Task Color_Sorter_Task;
extern pros::Task Drive_Controls_task;
extern pros::Task odom_task;
