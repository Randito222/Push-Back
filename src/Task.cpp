#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "subsystems.hpp"
// #include "pros/competition.hpp"

double Get_Color1() {
  return OP1.get_hue();
}
double Get_Color2() {
  return OP2.get_hue();
}


void Color_Mode() {
  bool lastButtonState = false;  // Tracks the last state of the button to detect presses

  while (true) {
    // Check if the Y button is currently pressed
    bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    // Only act when the button changes from not pressed to pressed (rising edge)
    if (currentButtonState && !lastButtonState) {
      colorMode = 1 - colorMode;  // Toggle between 0 (Red) and 1 (Blue)
    }

    // Update the last button state
    lastButtonState = currentButtonState;

    pros::delay(20); // Small delay to prevent CPU overload
  }
}

void Color_Sorter() {
  while (true) {
    Get_Color1();
    Get_Color2();

    if(colorMode == 0) { // Sorting based on Red
        if(BackIntakeControl == 1){

            // if(Get_Color1() < 100) { // Adjust threshold value as needed
            //   // Detected color is Red
            //   BackIntake.move(-127);
            //   pros::delay(500); // Run motor for 500 milliseconds
            // } else {
            //   BackIntake.move(127);
            // }
    
      
        } 
    }
    else { // Sorting based on Blue 
        if(BackIntakeControl == 1){

            // if(Get_Color2() > 200) { // Adjust threshold value as needed
            //   // Detected color is Blue
            //   BackIntake.move(-127);
            //   pros::delay(500); // Run motor for 500 milliseconds
            // } else {
            //   BackIntake.move(127);
            // }
    
      
        } 
    }
    pros::delay(20); // Small delay to prevent CPU overload
  }
}

void Drive_Controls_swap() {
  bool lastButtonState = false;  // Tracks the last state of the button to detect presses
  bool fieldCentric = true;     // Start in robot-centric mode

  while (true) {
    // Check if the X button is currently pressed
    bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);


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