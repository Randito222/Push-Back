#include "main.h"
#include "subsystems.hpp"
/*

Drive Controls

**/

void setDrivePower(int fl, int fr, int bl, int br) {
  // Set all motors for each wheel group
  Front_Left_1.move(fl);
  Front_Left_2.move(fl);

  Front_Right_1.move(fr);
  Front_Right_2.move(fr);

  Back_Left_1.move(bl);
  Back_Left_2.move(bl);

  Back_Right_1.move(br);
  Back_Right_2.move(br);
} 

// void IntakeSpin() {
//   // Spin the intake motor
//   Intake.move_velocity(200);  // Set the intake motor to spin at 200 RPM
// }

// Start by storing the robot's current heading as the initial target
double targetAngle = IMU.get_heading();
bool lastButtonState = false;  // Tracks the last state of the button to detect presses

void DriveControl() {

  // --- BUTTON PRESS HANDLING ---
  // Check if the X button is currently pressed
  bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);

  // Only act when the button changes from not pressed to pressed (rising edge)
  if (currentButtonState && !lastButtonState) {
    // Increment the target heading by 45 degrees
    targetAngle += 45.0;

    // Keep target heading in range [0, 360)
    if (targetAngle >= 360.0) targetAngle -= 360.0;
  }

  // Update the last button state
  lastButtonState = currentButtonState;

  // --- GET CURRENT IMU ANGLE AND CONVERT TO RADIANS ---
  double currentAngle = IMU.get_heading();
  double Radians = (M_PI / 180) * currentAngle;

  // --- READ CONTROLLER ANALOG INPUTS ---
  int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Forward/backward
  int strafe  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);    // Left/right
  int rotate_input = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // Joystick rotation

  // --- ROTATION OF JOYSTICK INPUT BASED ON FIELD-CENTRIC CONTROL ---
  int tempforward = forward * cos(Radians) + strafe * sin(Radians);
  int tempstrafe  = -forward * sin(Radians) + strafe * cos(Radians);

  // --- ROTATION CONTROL: ROTATE TO TARGET ANGLE ---
  double error = targetAngle - currentAngle;

  // Make sure we take the shortest path (handling wrap-around at 360°)
  if (error > 180)  error -= 360;
  if (error < -180) error += 360;

  // Simple proportional controller to rotate to the target heading
  double kP = 1.5;  // You can tune this value
  int rotationPower = error * kP;

  // Clamp rotation power to valid motor range
  if (rotationPower > 127)  rotationPower = 127;
  if (rotationPower < -127) rotationPower = -127;

  // If the robot is close enough to the target angle, allow joystick rotation
  if (fabs(error) < 1.5) {
    rotationPower = rotate_input;  // Let driver rotate manually
  }

  // --- HOLONOMIC DRIVE CALCULATION (X-DRIVE) ---
  int fl = tempforward + tempstrafe + rotationPower;  // Front left motor
  int fr = tempforward - tempstrafe - rotationPower;  // Front right motor
  int bl = tempforward - tempstrafe + rotationPower;  // Back left motor
  int br = tempforward + tempstrafe - rotationPower;  // Back right motor

  // Send power to the motors
  setDrivePower(fl, fr, bl, br);

  // Delay to avoid overloading the CPU
  pros::delay(10);
}

// void IntakeReverse(){
//   Intake.move_velocity(-200);
// }

int DescoreLV = -1;
void descoreLeft(){
  DescoreLV*=-1;

  if (DescoreLV==1){
    DescoreLeft.set_value(1);
  }

  else{
    DescoreLeft.set_value(0);
}
}
int DescoreRV = -1;
void descoreRight(){
  DescoreRV*=-1;

  if (DescoreRV==1){
    DescoreRight.set_value(1);
  }

  else{
    DescoreRight.set_value(0);
}
}
int ScoreP = -1;
void ScoringP(){
  ScoreP*=-1;

  if (ScoreP==1){
    ScorePiston.set_value(1);
  }

  else{
    ScorePiston.set_value(0);
}
}