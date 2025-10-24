#include "main.h"
#include "subsystems.hpp"

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

            if(Get_Color1() < 100) { // Adjust threshold value as needed
              // Detected color is Red
              BackIntake.move(-127);
              pros::delay(500); // Run motor for 500 milliseconds
            } else {
              BackIntake.move(127);
            }
    
      
        } 
    }
    else { // Sorting based on Blue 
        if(BackIntakeControl == 1){

            if(Get_Color2() > 200) { // Adjust threshold value as needed
              // Detected color is Blue
              BackIntake.move(-127);
              pros::delay(500); // Run motor for 500 milliseconds
            } else {
              BackIntake.move(127);
            }
    
      
        } 
    }
    pros::delay(20); // Small delay to prevent CPU overload
  }
}