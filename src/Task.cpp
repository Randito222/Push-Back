#include "Task.hpp"
#include "Functions.hpp"
#include "subsystems.hpp"

int colorMode = 0;
int BackIntakeControl = 0;

double Get_Color1() { return OP1.get_hue(); }
double Get_Color2() { return OP2.get_hue(); }

void Color_Mode() {
  bool last = false;
  while (true) {
    bool cur = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    if (cur && !last) colorMode ^= 1;
    last = cur;
    pros::delay(20);
  }
}

void Color_Sorter() {
  while (true) {
    if (BackIntakeControl) {
      (colorMode == 0 ? Get_Color1() : Get_Color2());
    }
    pros::delay(20);
  }
}

void Drive_Controls_swap() {
  bool fieldCentric = true;
  bool last = false;

  while (true) {
    bool cur = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (cur && !last) fieldCentric = !fieldCentric;
    last = cur;

    fieldCentric ? DriveControl() : DriveControlBackUp();
    pros::delay(20);
  }
}

pros::Task Color_Mode_Task(Color_Mode);
pros::Task Color_Sorter_Task(Color_Sorter);
pros::Task Drive_Controls_task(Drive_Controls_swap);
