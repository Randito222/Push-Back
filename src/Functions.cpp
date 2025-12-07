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

int slewDrive(int target, int current, int rate) {
    if (current < target)
        current += rate;
    else if (current > target)
        current -= rate;

    // Snap when close
    if (abs(target - current) < rate)
        current = target;

    return current;
}

void DriveControl() {

  // =============================
    // Persistent power values
    // =============================
    static int flPower = 0;
    static int frPower = 0;
    static int blPower = 0;
    static int brPower = 0;

    const int slewRate = 50;   // Lower = smoother, higher = more responsive

    // =============================
    // Controller input with deadzones
    // =============================
    double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double strafe  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    double rotate  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (fabs(forward) < 5) forward = 0;
    if (fabs(strafe)  < 5) strafe  = 0;
    if (fabs(rotate)  < 5) rotate  = 0;

    // =============================
    // FIELD CENTRIC TRANSFORMATION
    // =============================
    double headingRad = IMU.get_rotation() * M_PI / 180.0;

    double tempForward =  forward * cos(headingRad) + strafe * sin(headingRad);
    double tempStrafe  = -forward * sin(headingRad) + strafe * cos(headingRad);

    forward = tempForward;
    strafe  = tempStrafe;

    // =============================
    // X-DRIVE MOTOR MIXING
    // =============================
    int flTarget = forward + strafe + rotate;
    int frTarget = forward - strafe - rotate;
    int blTarget = forward - strafe + rotate;
    int brTarget = forward + strafe - rotate;

    // =============================
    // INTERNAL SLEW RATE LIMITING
    // =============================
    auto applySlew = [&](int target, int &current) {
        if (current < target)
            current += slewRate;
        else if (current > target)
            current -= slewRate;

        // If close, snap to target
        if (abs(target - current) < slewRate)
            current = target;
    };

    applySlew(flTarget, flPower);
    applySlew(frTarget, frPower);
    applySlew(blTarget, blPower);
    applySlew(brTarget, brPower);

    // =============================
    // Send power to motors
    // =============================
    setDrivePower(flPower, frPower, blPower, brPower);

    pros::delay(10);
}


// void IntakeReverse(){
//   Intake.move_velocity(-200);
// }

int DescoreLV = -1;
void descoreLeftT(){
  DescoreLV*=-1;

  if (DescoreLV==1){
    DescoreLeft.set_value(1);
  }

  else{
    DescoreLeft.set_value(0);
}
}
// int DescoreRV = -1;
// void descoreRight(){
//   DescoreRV*=-1;

//   if (DescoreRV==1){
//     DescoreRight.set_value(1);
//   }

//   else{
//     DescoreRight.set_value(0);
// }
// }

// int ScoreP = -1;
// void ScoringP(){
//   ScoreP*=-1;

//   if (ScoreP==1){
//     ScorePiston.set_value(1);
//   }

//   else{
//     ScorePiston.set_value(0);
// }
// }

int IntakeScoreV = -1;
void IntakeScoreToggle(){
  IntakeScoreV*=-1;

  if (IntakeScoreV==1){
    IntakeLift.set_value(1);
  }

  else{
    IntakeLift.set_value(0);
  }
}

int MatchLoadV = -1;
void MatchLoading(){
  MatchLoadV*=-1;

  if (MatchLoadV==1){
    TongueMech.set_value(1);
  }

  else{
    TongueMech.set_value(0);
  }
}

void ArmAction(){
  if(master.get_digital(DIGITAL_B) == true) {
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-530,30);  // Stop the intake motor when B is pressed
    KnownState=1;
    pros::delay(200);
    FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
    }
  else if (master.get_digital(DIGITAL_B) == false && KnownState == 1){ 
    Arm.move_absolute(5,200);  // Stop the intake motor when B is released
    KnownState=0;
    pros::delay(200);

  }
}