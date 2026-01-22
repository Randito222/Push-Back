#pragma once
#include "pros/rtos.hpp"
<<<<<<< HEAD

// =============================
// Color Sorting
// =============================
double Get_Color1();
double Get_Color2();

void Color_Mode();
void Color_Sorter();
=======
double Get_Color1(); // gets color value from optical sensor 1
double Get_Color2(); // gets color value from optical sensor 2

void Color_Mode(); // changes color mode based on button press
void Color_Sorter(); // task function for color sorting
void Drive_Controls_swap(); // swaps between field centric and robot centric drive
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9

// =============================
// Drive Mode
// =============================
void Drive_Controls_swap();

<<<<<<< HEAD
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
=======
inline pros::Task Color_Mode_Task(Color_Mode);
inline pros::Task Color_Sorter_Task(Color_Sorter);
inline pros::Task Drive_Controls_task(Drive_Controls_swap);

>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
