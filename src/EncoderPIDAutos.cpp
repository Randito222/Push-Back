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

// ============================
// Horizontal tracking wheel
// ============================

constexpr double H_WHEEL_DIAM_IN = 2.0;
constexpr double H_WHEEL_CIRC_IN = M_PI * H_WHEEL_DIAM_IN;

// Horizontal wheel offset from robot center
// + if in FRONT of center, - if BEHIND center
constexpr double H_OFFSET_IN = -3.0;

// ============================
// Helpers
// ============================

static inline double rotCentiDegToIn(double centiDeg) {
  return (centiDeg / 100.0) / 360.0 * H_WHEEL_CIRC_IN;
}

static inline double getHorizontalIn() {
  return rotCentiDegToIn((double)HorizontalTracker.get_position());
}

static inline void resetHorizontalTracker() {
  HorizontalTracker.reset_position();
  pros::delay(5);
}


// ============================
// DRIVE FORWARD PID
// ============================

void driveForward_EncoderPID(double targetInches,
                             double holdHeadingDeg,
                             int maxDrive,
                             int maxTurn,
                             int timeoutMs) {

  const double wheelDiamIn = 3.25;

  // =============================
  // Translation PID (tuned to reduce overshoot + back-up hunting)
  // =============================
  const double Kp = 0.24;
  const double Ki = 0.0015;   // smaller I to reduce "push past target"
  const double Kd = 0.70;     // more damping

  // Only allow integral near the end (prevents windup)
  const double integralActiveZoneDeg = inToDeg(6.0, wheelDiamIn);     // last 6 inches
  const double integralLimit = (Ki > 0.0) ? (25.0 / Ki) : 0.0;        // ~25 power max from I

  // Exit + settle (prevents "fly-by then reverse" oscillation)
  const double exitThresholdDeg = inToDeg(0.5, wheelDiamIn);         
  const double settlePosTolDeg  = inToDeg(0.6, wheelDiamIn);          // slightly looser than exit
  const int    settleCyclesReq  = 8;                                  // 8*20ms = 160ms

  // =============================
  // Heading hold PD
  // =============================
  const double kP_h = 2.0; 
  const double kD_h = 6.0;
  const double MIN_T = 4.0;
  const double headTolDeg = 1.0;

  // =============================
  // Strafe correction PD (horizontal tracker)
  // =============================
  const double kP_x = 20;     
  const double kD_x = 0.0;    
  const double xMax = 45.0;
  const double strafeDeadbandIn = 0.05;

  double error = 0, lastError = 0;
  double integral = 0;
  double lastHeadErr = 0;
  double lastStrafeErr = 0;

  int settleCount = 0;

  resetDriveEncoders();
  resetHorizontalTracker();

  const uint32_t start = pros::millis();

  const double targetDeg = inToDeg(targetInches, wheelDiamIn);

  const double startH = getHorizontalIn();
  const double startHeadingDeg = IMU.get_rotation();

  while (true) {

    if ((int)(pros::millis() - start) > timeoutMs) break;

    // =============================
    // Translation error
    // =============================
    const double posDeg = getDriveAvgDegForward();
    error = targetDeg - posDeg;

    // =============================
    // Heading hold
    // =============================
    const double curHeading = IMU.get_rotation();
    const double headErr = wrapDeg(holdHeadingDeg - curHeading);

    // Settle-based exit (reduces "overshoot then back up")
    if (std::fabs(error) < settlePosTolDeg && std::fabs(headErr) < headTolDeg) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleCyclesReq) break;

    // =============================
    // Integral gating + clamp (prevents windup)
    // =============================
    if (std::fabs(error) < integralActiveZoneDeg && std::fabs(error) > exitThresholdDeg) {
      integral += error;
    } else {
      integral = 0;
    }
    if (Ki > 0.0) integral = clampd(integral, -integralLimit, integralLimit);

    // Derivative
    double derivative = error - lastError;
    lastError = error;

    // Translation output (y)
    double yOut = (Kp * error) + (Ki * integral) + (Kd * derivative);
    yOut = clampd(yOut, -std::fabs((double)maxDrive), std::fabs((double)maxDrive));

    // Heading output (r)
    double rOut = (kP_h * headErr) + (kD_h * (headErr - lastHeadErr));
    lastHeadErr = headErr;

    rOut = clampd(rOut, -std::fabs((double)maxTurn), std::fabs((double)maxTurn));
    if (std::fabs(headErr) > headTolDeg && std::fabs(rOut) < MIN_T)
      rOut = (headErr > 0 ? MIN_T : -MIN_T);

    // =============================
    // Horizontal tracking correction
    // =============================
    const double hNow = getHorizontalIn();
    const double headingChangeDeg = curHeading - startHeadingDeg;
    const double headingChangeRad = headingChangeDeg * M_PI / 180.0;

    // Remove rotation-induced motion
    double correctedStrafe =
        (hNow - startH) - (headingChangeRad * H_OFFSET_IN);

    double strafeErr = -correctedStrafe;

    double xOut =
        (kP_x * strafeErr) +
        (kD_x * (strafeErr - lastStrafeErr));

    lastStrafeErr = strafeErr;

    if (std::fabs(correctedStrafe) < strafeDeadbandIn) xOut = 0;
    xOut = clampd(xOut, -xMax, xMax);

    // =============================
    // X-drive mix
    // =============================
    double fl = yOut + xOut + rOut;
    double fr = yOut - xOut - rOut + 20;
    double bl = yOut - xOut + rOut + 20;
    double br = yOut + xOut - rOut;

    double maxMag = std::max({std::fabs(fl),
                              std::fabs(fr),
                              std::fabs(bl),
                              std::fabs(br)});

    if (maxMag > 127.0) {
      double scale = 127.0 / maxMag;
      fl *= scale;
      fr *= scale;
      bl *= scale;
      br *= scale;
    }

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

void Drive_EncoderPID(double targetInchesY,
                                         double targetInchesX,
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

  double errorY = 0, lastErrorY = 0;
  double errorX = 0, LastErrorX = 0;
  double integralY = 0, derivativeY = 0;
  double integralX =0, derivativeX =0;

  double lastHeadErr = 0;

  const double integralActiveZoneDeg = inToDeg(15.0, wheelDiamIn);
  const double integralPowerLimit = (Ki > 0.0) ? (40.0 / Ki) : 0.0;

  const double exitThresholdDeg = inToDeg(0.5, wheelDiamIn);

  resetDriveEncoders();
  const uint32_t start = pros::millis();

  const double targetDegY = inToDeg(targetInchesY * change, wheelDiamIn);
  const double targetDegX = inToDeg(targetInchesX * change, wheelDiamIn);

  while (true) {
    if ((int)(pros::millis() - start) > timeoutMs) break;

    // --- Translation error ---
    const double posDegY = getDriveAvgDegForward();
    const double posDegX = getDriveAvgDegStrafe();
    errorY = targetDegY - posDegY;
    errorX = targetDegX - posDegX;

    // --- Heading error ---
    const double curHeading = IMU.get_rotation();
    const double headErr = wrapDeg(holdHeadingDeg - curHeading);

    // Exit when close in distance AND close in heading
    if (std::fabs(errorY) < exitThresholdDeg && std::fabs(errorX) < exitThresholdDeg && std::fabs(headErr) < headTolDeg) break;

    // Integral active zone
    if (std::fabs(errorY) < integralActiveZoneDeg && errorY != 0) integralY += errorY;
    else integralY = 0;

    if (std::fabs(errorX) < integralActiveZoneDeg && errorX != 0) integralX += errorX;
    else integralX = 0;

    if (Ki > 0.0) integralY = clampd(integralY, -integralPowerLimit, integralPowerLimit);
    if (Ki > 0.0) integralX = clampd(integralX, -integralPowerLimit, integralPowerLimit);

    derivativeY = errorY - lastErrorY;
    lastErrorY = errorY;

    derivativeX = errorX - LastErrorX;
    LastErrorX = errorX;

    // Translation output (y)
    double yOut = (Kp * errorY) + (Ki * integralY) + (Kd * derivativeY);
    yOut = clampd(yOut, -std::fabs((double)maxDrive), std::fabs((double)maxDrive));

    // Heading output (r) - PD
    double rOut = (kP_h * headErr) + (kD_h * (headErr - lastHeadErr));
    lastHeadErr = headErr;

    rOut = clampd(rOut, -std::fabs((double)maxTurn), std::fabs((double)maxTurn));
    if (std::fabs(headErr) > headTolDeg && std::fabs(rOut) < MIN_T) rOut = (headErr > 0 ? MIN_T : -MIN_T);

    // Strafe output (x)
    double xOut = (Kp * errorX) + (Ki * integralX) + (Kd * derivativeX);
    yOut = clampd(xOut, -std::fabs((double)maxDrive), std::fabs((double)maxDrive));

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

// =============================
// Wheel geometry (inches)
// =============================
constexpr double VERT_DIAM_IN = 2.75;
constexpr double HORZ_DIAM_IN = 2.00;

// =============================
// ODOM OFFSETS (inches)
// =============================
// Distance between L/R vertical trackers (used for wheel-based heading if you want it later)
constexpr double TRACK_WIDTH_IN = 6.0;

// =============================
// Utility helpers
// =============================

static inline double centiDegToIn(double centiDeg, double wheelDiamIn) {
  // Rotation sensor get_position() returns centidegrees (0.01 deg)
  return (centiDeg / 100.0) / 360.0 * (M_PI * wheelDiamIn);
}

// LEFT vertical is flipped here (IMPORTANT)
static inline double getLVerticalIn() {
  return -centiDegToIn((double)LVerticalTracker.get_position(), VERT_DIAM_IN);
}

static inline double getRVerticalIn() {
  return  centiDegToIn((double)RVerticalTracker.get_position(), VERT_DIAM_IN);
}

inline void resetTrackers() {
  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();
  pros::delay(5);
}


// =============================
// DRIVE FORWARD (Tracking-wheel based) PID
// Uses: L/R vertical for distance, IMU for heading, horizontal wheel for anti-drift
// =============================
void driveForward_EncoderPID2(double targetInches,
                              double holdHeadingDeg,
                              int maxDrive,
                              int maxTurn,
                              int timeoutMs) {

  // -----------------------------
  // Optional scale (ONLY for distance calibration, not diagonal)
  // -----------------------------
  const double L_scale = 0.98;
  const double R_scale = 1.00;

  // -----------------------------
  // Translation PID (INCHES)
  // -----------------------------
  const double Kp = 8.0;
  const double Ki = 0.02;
  const double Kd = 25.0;

  const double integralActiveZoneIn = 6.0;
  const double integralLimit = 40.0;
  const double posTolIn = 0.5;

  // -----------------------------
  // Heading hold PD (IMU)
  // -----------------------------
  const double kP_h = 2.5;
  const double kD_h = 6.0;
  const double headTolDeg = 1.0;
  const double MIN_T_MOVING = 6.0;

  // -----------------------------
  // Strafe hold PD (horizontal wheel)
  // (FIX: calmer values; 30 was too aggressive and can cause diagonal snaps)
  // -----------------------------
  const double kP_x = 10.0;
  const double kD_x = 60.0;
  const double strafeDeadbandIn = 0.10;

  // -----------------------------
  // Per-loop pod balance (vertical mismatch -> rotate correction)
  // (FIX: use diffStep, NOT cumulative LIn-RIn)
  // -----------------------------
  const double balanceMinY = 15.0;     // only apply when moving
  const double balanceK_step = 250.0;  // tune 150..450 (diffStep is tiny)

  // -----------------------------
  // Settle exit (prevents "ends diagonal")
  // -----------------------------
  const double strafeTolIn     = 0.25;
  const int    settleCyclesReq = 8;
  int settleCount = 0;

  resetTrackers();

  double error = 0, lastError = 0;
  double integral = 0;

  double lastHeadErr = 0;
  double lastStrafeErr = 0;

  const double startHeadingDeg = IMU.get_heading();
  const double startH = getHorizontalIn();

  const uint32_t startTime = pros::millis();

  // Filter state resets once per call
  static double filtStrafe = 0.0;
  filtStrafe = 0.0;

  // Per-loop baseline for balance (must reset once per call)
  double lastLIn_step = getLVerticalIn() * L_scale;
  double lastRIn_step = getRVerticalIn() * R_scale;

  int lcdCounter = 0;

  while (true) {
    const uint32_t now = pros::millis();
    if ((int)(now - startTime) > timeoutMs) break;

    // =============================
    // Forward distance from vertical tracking wheels
    // =============================
    const double LIn = getLVerticalIn() * L_scale;
    const double RIn = getRVerticalIn() * R_scale;
    const double forwardIn = (LIn + RIn) / 2.0;
    error = targetInches - forwardIn;

    // =============================
    // Per-loop mismatch for pod-balance (stable)
    // =============================
    const double dL_step = LIn - lastLIn_step;
    const double dR_step = RIn - lastRIn_step;
    lastLIn_step = LIn;
    lastRIn_step = RIn;

    const double diffStep = (dL_step - dR_step); // + means left advanced more THIS LOOP

    // =============================
    // Heading hold (IMU)
    // =============================
    const double curHeading = IMU.get_heading();
    const double headErr = wrapDeg(holdHeadingDeg - curHeading);

    // =============================
    // Integral gating + clamp
    // =============================
    if (std::fabs(error) < integralActiveZoneIn) integral += error;
    else integral = 0;
    integral = clampd(integral, -integralLimit, integralLimit);

    // Derivative
    const double derivative = error - lastError;
    lastError = error;

    // Translation output (y)
    double yOut = (Kp * error) + (Ki * integral) + (Kd * derivative);
    yOut = clampd(yOut, -(double)maxDrive, (double)maxDrive);

    // =============================
    // Heading output (r)
    // =============================
    double rOut = (kP_h * headErr) + (kD_h * (headErr - lastHeadErr));
    lastHeadErr = headErr;

    // Scale heading correction slightly with forward speed
    double speedScale = 0.5 + (std::fabs(yOut) / std::max(1.0, (double)maxDrive)) * 0.7;
    rOut *= speedScale;

    // -----------------------------
    // POD BALANCE (per-loop) — prevents diagonal from friction mismatch
    // -----------------------------
    if (std::fabs(yOut) > balanceMinY) {
      double balanceOut = balanceK_step * diffStep;

      // reduce balance near the end so it doesn't twist while stopping
      if (std::fabs(error) < 3.0) balanceOut *= 0.3;

      // If it gets worse, flip sign on this line:
      rOut -= balanceOut;
      // rOut += balanceOut;
    }

    rOut = clampd(rOut, -(double)maxTurn, (double)maxTurn);

    // Minimum turn while moving (beats pod friction)
    if (std::fabs(headErr) > headTolDeg && std::fabs(yOut) > 20 && std::fabs(rOut) < MIN_T_MOVING) {
      rOut = (headErr > 0 ? MIN_T_MOVING : -MIN_T_MOVING);
    }

    // =============================
    // Horizontal anti-drift (strafe hold)
    // =============================
    const double hNow = getHorizontalIn();

    const double headingChangeDeg = curHeading - startHeadingDeg;
    const double headingChangeRad = headingChangeDeg * M_PI / 180.0;

    // Remove rotation-induced horizontal wheel motion due to offset
    const double correctedStrafe =
      (hNow - startH) - (headingChangeRad * H_OFFSET_IN);

    // Low-pass filter to reduce noise
    const double alpha = 0.3;
    filtStrafe = (alpha * correctedStrafe) + (1.0 - alpha) * filtStrafe;

    const double strafeErr = -filtStrafe;

    double xOut = (kP_x * strafeErr) + (kD_x * (strafeErr - lastStrafeErr));
    lastStrafeErr = strafeErr;

    // Ignore strafe correction briefly (pods settling / wheel contact)
    if (now - startTime < 100) xOut = 0;

    // Deadband uses filtered value
    if (std::fabs(filtStrafe) < strafeDeadbandIn) xOut = 0;

    // Dynamic strafe authority (stronger at high speed)
    double xMaxNow =
        20.0 + 30.0 * (std::fabs(yOut) / std::max(1.0, (double)maxDrive)); // 20..50
    xOut = clampd(xOut, -xMaxNow, xMaxNow);

    // =============================
    // Settle-based exit (distance + heading + strafe)
    // =============================
    const bool posOK    = (std::fabs(error) < posTolIn);
    const bool headOK   = (std::fabs(headErr) < headTolDeg);
    const bool strafeOK = (std::fabs(filtStrafe) < strafeTolIn);

    if (posOK && headOK && strafeOK) settleCount++;
    else settleCount = 0;

    if (settleCount >= settleCyclesReq) break;

    // =============================
    // PID DEBUG PRINT (Brain LCD)
    // =============================
    if (++lcdCounter >= 5) { // ~100ms
      lcdCounter = 0;

      pros::lcd::print(0, "Fwd:%.2f Err:%.2f", forwardIn, error);
      pros::lcd::print(1, "Head:%.1f E:%.1f", curHeading, headErr);
      pros::lcd::print(2, "Str:%.2f Xerr:%.2f", filtStrafe, strafeErr);
      pros::lcd::print(3, "Y:%.0f X:%.0f R:%.0f", yOut, xOut, rOut);
      pros::lcd::print(4, "L:%.2f R:%.2f dS:%.3f", LIn, RIn, diffStep);
      pros::lcd::print(5, "Set:%d t:%dms", settleCount, (int)(now - startTime));
    }

    // =============================
    // X-drive mix
    // =============================
    double fl = yOut + xOut + rOut;
    double fr = yOut - xOut - rOut;
    double bl = yOut - xOut + rOut;
    double br = yOut + xOut - rOut;

    double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br)});
    if (maxMag > 127.0) {
      double scale = 127.0 / maxMag;
      fl *= scale; fr *= scale; bl *= scale; br *= scale;
    }

    setDrivePower((int)fl, (int)fr, (int)bl, (int)br);
    pros::delay(20);
  }

  setDrivePower(0, 0, 0, 0);
  pros::lcd::print(5, "DONE err=%.2f set=%d", error, settleCount);
}