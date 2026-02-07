#include "main.h"

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline double degToIn(double deg, double wheelDiamIn) {
  return (deg / 360.0) * (M_PI * wheelDiamIn);
}

static inline double inToDeg(double inches, double wheelDiamIn) {
  return (inches / (M_PI * wheelDiamIn)) * 360.0;
}

// Reset ONLY drive motor encoders
void resetDriveEncoders() {
  Front_Left_1.tare_position();  Front_Left_2.tare_position();
  Front_Right_1.tare_position(); Front_Right_2.tare_position();
  Back_Left_1.tare_position();   Back_Left_2.tare_position();
  Back_Right_1.tare_position();  Back_Right_2.tare_position();
}

// Average “forward distance” in motor degrees (good for forward/back)
double getDriveAvgDegForward() {
  // Average each wheel group (2 motors), then average 4 groups
  double fl = (Front_Left_1.get_position()  + Front_Left_2.get_position())  * 0.5;
  double fr = (Front_Right_1.get_position() + Front_Right_2.get_position()) * 0.5;
  double bl = (Back_Left_1.get_position()   + Back_Left_2.get_position())   * 0.5;
  double br = (Back_Right_1.get_position()  + Back_Right_2.get_position())  * 0.5;

  // For forward, all wheels should contribute similarly (signs should already be handled by motor reversal)
  return (fl + fr + bl + br) * 0.25;
}

// “Strafe distance” estimate in motor degrees (x-drive specific)
// For an X-drive, strafe right typically has FL+BR forward, FR+BL backward.
// This combination extracts the strafe component from wheel rotations.
double getDriveAvgDegStrafe() {
  double fl = (Front_Left_1.get_position()  + Front_Left_2.get_position())  * 0.5;
  double fr = (Front_Right_1.get_position() + Front_Right_2.get_position()) * 0.5;
  double bl = (Back_Left_1.get_position()   + Back_Left_2.get_position())   * 0.5;
  double br = (Back_Right_1.get_position()  + Back_Right_2.get_position())  * 0.5;

  // Strafe component (right +) using typical x-drive sign pattern
  // If strafe direction is reversed on your bot, just negate the return.
  return (fl - fr - bl + br) * 0.25;
}

static inline double wrapDeg(double a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void driveForward_EncoderPID(double targetInches,
                                         double holdHeadingDeg,
                                         int maxDrive,
                                         int maxTurn,
                                         int timeoutMs) {
  const double wheelDiamIn = 3.25;  // DRIVE wheel diameter
  const double change = 1.0;

  // Translation PID
  const double Kp = 0.30;
  const double Ki = 0.02;
  const double Kd = 0.50;

  // Heading hold PD (start small)
  const double kP_h = 2.0;   // deg -> power
  const double kD_h = 6.0;
  const double MIN_T = 4.0;
  const double headTolDeg = 2.0;

  double error = 0, lastError = 0;
  double integral = 0, derivative = 0;

  double lastHeadErr = 0;

  const double integralActiveZoneDeg = inToDeg(15.0, wheelDiamIn);
  const double integralPowerLimit = (Ki > 0.0) ? (40.0 / Ki) : 0.0;

  const double exitThresholdDeg = inToDeg(0.5, wheelDiamIn);

  resetDriveEncoders();
  const uint32_t start = pros::millis();

  const double targetDeg = inToDeg(targetInches * change, wheelDiamIn);

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    // --- Translation error ---
    const double posDeg = getDriveAvgDegForward();
    error = targetDeg - posDeg;

    // --- Heading error ---
    const double curHeading = IMU.get_rotation();
    const double headErr = wrapDeg(holdHeadingDeg - curHeading);

    // Exit when close in distance AND close in heading
    if (std::fabs(error) < exitThresholdDeg && std::fabs(headErr) < headTolDeg) break;

    // Integral active zone
    if (std::fabs(error) < integralActiveZoneDeg && error != 0) integral += error;
    else integral = 0;
    if (Ki > 0.0) integral = clampd(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    // Translation output (y)
    double yOut = (Kp * error) + (Ki * integral) + (Kd * derivative);
    yOut = clampd(yOut, -std::fabs((double)maxDrive), std::fabs((double)maxDrive));

    // Heading output (r) - PD
    double rOut = (kP_h * headErr) + (kD_h * (headErr - lastHeadErr));
    lastHeadErr = headErr;

    rOut = clampd(rOut, -std::fabs((double)maxTurn), std::fabs((double)maxTurn));
    if (std::fabs(headErr) > headTolDeg && std::fabs(rOut) < MIN_T) rOut = (headErr > 0 ? MIN_T : -MIN_T);

    // No strafe
    double xOut = 0;

    // X-drive mix
    double fl = yOut + xOut + rOut;
    double fr = yOut - xOut - rOut;
    double bl = yOut - xOut + rOut;
    double br = yOut + xOut - rOut;

    // Normalize
    const double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br), 127.0});
    fl = fl * 127.0 / maxMag;
    fr = fr * 127.0 / maxMag;
    bl = bl * 127.0 / maxMag;
    br = br * 127.0 / maxMag;

    setDrivePower((int)fl, (int)fr, (int)bl, (int)br);
    pros::delay(20);
  }

  setDrivePower(0,0,0,0);
}


void driveStrafe_EncoderPID(double targetInches,
                                        double holdHeadingDeg,
                                        int maxDrive,
                                        int maxTurn,
                                        int timeoutMs) {
  const double wheelDiamIn = 3.25;  // DRIVE wheel diameter
  const double change = 1.0;

  const double Kp = 0.30;
  const double Ki = 0.02;
  const double Kd = 0.50;

  const double kP_h = 2.0;
  const double kD_h = 6.0;
  const double MIN_T = 4.0;
  const double headTolDeg = 2.0;

  double error = 0, lastError = 0;
  double integral = 0, derivative = 0;
  double lastHeadErr = 0;

  const double integralActiveZoneDeg = inToDeg(15.0, wheelDiamIn);
  const double integralPowerLimit = (Ki > 0.0) ? (40.0 / Ki) : 0.0;

  const double exitThresholdDeg = inToDeg(0.5, wheelDiamIn);

  resetDriveEncoders();
  const uint32_t start = pros::millis();

  const double targetDeg = inToDeg(targetInches * change, wheelDiamIn);

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    const double posDeg = getDriveAvgDegStrafe();
    error = targetDeg - posDeg;

    const double curHeading = IMU.get_rotation();
    const double headErr = wrapDeg(holdHeadingDeg - curHeading);

    if (std::fabs(error) < exitThresholdDeg && std::fabs(headErr) < headTolDeg) break;

    if (std::fabs(error) < integralActiveZoneDeg && error != 0) integral += error;
    else integral = 0;
    if (Ki > 0.0) integral = clampd(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    // Strafe output (x)
    double xOut = (Kp * error) + (Ki * integral) + (Kd * derivative);
    xOut = clampd(xOut, -std::fabs((double)maxDrive), std::fabs((double)maxDrive));

    // Heading output (r)
    double rOut = (kP_h * headErr) + (kD_h * (headErr - lastHeadErr));
    lastHeadErr = headErr;

    rOut = clampd(rOut, -std::fabs((double)maxTurn), std::fabs((double)maxTurn));
    if (std::fabs(headErr) > headTolDeg && std::fabs(rOut) < MIN_T) rOut = (headErr > 0 ? MIN_T : -MIN_T);

    double yOut = 0;

    double fl = yOut + xOut + rOut;
    double fr = yOut - xOut - rOut;
    double bl = yOut - xOut + rOut;
    double br = yOut + xOut - rOut;

    const double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br), 127.0});
    fl = fl * 127.0 / maxMag;
    fr = fr * 127.0 / maxMag;
    bl = bl * 127.0 / maxMag;
    br = br * 127.0 / maxMag;

    setDrivePower((int)fl, (int)fr, (int)bl, (int)br);
    pros::delay(20);
  }

  setDrivePower(0,0,0,0);
}


void turnToHeading_IMUPID(double targetDeg, int maxPower, int timeoutMs) {
  const double Kp = 1.5;
  const double Ki = 0.0;
  const double Kd = 8.0;

  double error = 0, lastError = 0;
  double integral = 0, derivative = 0;

  const double integralActiveZone = 15.0;
  const double integralPowerLimit = (Ki > 0.0) ? (40.0 / Ki) : 0.0;

  const double exitThreshold = 1.5;

  const uint32_t start = pros::millis();

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    double cur = IMU.get_rotation();
    error = wrapDeg(targetDeg - cur);

    if (std::fabs(error) < exitThreshold) break;

    if (std::fabs(error) < integralActiveZone && error != 0) integral += error;
    else integral = 0;

    if (Ki > 0.0) integral = clampd(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    double out = (Kp * error) + (Ki * integral) + (Kd * derivative);
    out = clampd(out, -std::fabs((double)maxPower), std::fabs((double)maxPower));

    // pure rotate
    setDrivePower((int)out, (int)-out, (int)out, (int)-out);

    pros::delay(20);
  }

  setDrivePower(0,0,0,0);
}