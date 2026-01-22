<<<<<<< HEAD
#include "Task.hpp"
#include "Functions.hpp"
=======
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
#include "subsystems.hpp"
// #include "pros/competition.hpp"

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

<<<<<<< HEAD
    fieldCentric ? DriveControl() : DriveControlBackUp();
    pros::delay(20);
  }
}

pros::Task Color_Mode_Task(Color_Mode);
pros::Task Color_Sorter_Task(Color_Sorter);
pros::Task Drive_Controls_task(Drive_Controls_swap);
=======

    // Only act when the button changes from not pressed to pressed (rising edge)
    if (currentButtonState && !lastButtonState) {
      fieldCentric = !fieldCentric;  // Toggle between field-centric and robot-centric
      //master.rumble(fieldCentric ? "." : "..");
    }
    if (fieldCentric) {
      // Field-centric drive code
      // (This would involve transforming joystick inputs based on robot heading)
      DriveControl();
      master.clear_line(10);
      master.print(10, 0, "Field-Centric");
    } else {
      // Robot-centric drive code
      // (This would use joystick inputs directly)
      DriveControlBackUp();
      master.clear_line(10);
      master.print(10, 0, "Robot-Centric");
    }

    // Update the last button state
    lastButtonState = currentButtonState;

    pros::delay(20); // Small delay to prevent CPU overload
  }
}
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
