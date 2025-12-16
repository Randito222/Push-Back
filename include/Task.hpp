#include "pros/rtos.hpp"
#include "OdomSet.hpp"

double Get_Color1(); // gets color value from optical sensor 1
double Get_Color2(); // gets color value from optical sensor 2

void Color_Mode(); // changes color mode based on button press
void Color_Sorter(); // task function for color sorting
void Drive_Controls_swap(); // swaps between field centric and robot centric drive
void odomTask(); // task function for odometry update

static int colorMode = 0;      // 0 for sorting based on sensor 1, 1 for sensor 2
inline int BackIntakeControl = 0;

inline pros::Task Color_Mode_Task(Color_Mode);
inline pros::Task Color_Sorter_Task(Color_Sorter);
inline pros::Task Drive_Controls_task(Drive_Controls_swap);
//inline pros::Task odom_task(odomTask, "Odometry Task");
