#include "Odom.hpp"
#include "subsystems.hpp"
#include <cmath>

// =============================
// Wheel geometry (inches)
// =============================
constexpr double VERT_DIAM_IN = 2.75;
constexpr double HORZ_DIAM_IN = 2.00;

// =============================
// ODOM OFFSETS (inches)
// =============================
// Distance between L/R vertical trackers (only needed for wheel-based heading)
constexpr double TRACK_WIDTH_IN = 6.0;

// Horizontal wheel offset from robot center
// + if in FRONT of center, - if BEHIND center
constexpr double H_OFFSET_IN = -3.0;

// =============================
// Global pose
// =============================
double odomX = 0.0;
double odomY = 0.0;
double odomTheta = 0.0;

// =============================
// RESET LATCH (prevents delta spikes when trackers reset elsewhere)
// =============================
// If you reset Rotation sensors in another thread/function (auton PID, etc.),
// odomTask can see a huge "jump" for one cycle. This latch lets odomTask
// re-baseline cleanly before computing deltas.
static volatile bool gOdomResetLatch = false;

static inline void odomLatchResetRequest() {
  gOdomResetLatch = true;
}

// =============================
// Helpers
// =============================
static inline double wrapPi(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double degToRad(double d) {
  return d * M_PI / 180.0;
}

// Rotation sensor returns hundredths of a degree. Convert deg -> inches for wheel travel.
static inline double degToIn(double deg, double wheelDiamIn) {
  return (deg / 36000.0) * (M_PI * wheelDiamIn);
}

// =============================
// SIGN SETTINGS (tune once)
// =============================
// Start all at +1, then push the robot by hand to verify:
// - Push forward: odomY should increase (if not, flip both LV/RV)
// - Push right:   odomX should increase (if not, flip H)
static int SIGN_LV = +1;
static int SIGN_RV = +1;
static int SIGN_H  = +1;


void odomReset(double xIn, double yIn) {
  odomLatchResetRequest();

  odomX = xIn;
  odomY = yIn;

  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  //IMU.reset();

  odomTheta = -wrapPi(degToRad(IMU.get_rotation()));
}

void odomTask() {
  int lcdCounter = 0;
  int usbCounter = 0;

  // // Wait for IMU calibration
  // while (IMU.is_calibrating()) pros::delay(10);

  // Initialize with the SAME sign convention used in the loop
  double lastLdeg = -LVerticalTracker.get_position();
  double lastRdeg =  RVerticalTracker.get_position();
  double lastHdeg =  HorizontalTracker.get_position();

  double lastHeadingRad = wrapPi(degToRad(IMU.get_rotation()));
  odomTheta = lastHeadingRad;

  const int loopMs = 15;

  // Helper clamp (local)
  auto clampd_local = [](double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  };

  while (true) {

    // =============================
    // RESET LATCH HANDLING
    // =============================
    if (gOdomResetLatch) {
      // Re-sample baselines AFTER the reset so next deltas are ~0
      lastLdeg = -LVerticalTracker.get_position();
      lastRdeg =  RVerticalTracker.get_position();
      lastHdeg =  HorizontalTracker.get_position();

      lastHeadingRad = wrapPi(degToRad(IMU.get_rotation()));
      odomTheta = lastHeadingRad;

      gOdomResetLatch = false;
      pros::delay(loopMs);
      continue;
    }

    // --- Wheel positions (deg/centideg units from Rotation;) ---
    const double Ldeg = -LVerticalTracker.get_position();
    const double Rdeg =  RVerticalTracker.get_position();
    const double Hdeg =  HorizontalTracker.get_position();

    // --- Heading from IMU (radians) ---
    const double headingRad = -wrapPi(degToRad(IMU.get_rotation()));
    const double dThetaRaw  = wrapPi(headingRad - lastHeadingRad);

    // --- Delta inches (apply sign) ---
    const double dLraw = SIGN_LV * degToIn(Ldeg - lastLdeg, VERT_DIAM_IN);
    const double dRraw = SIGN_RV * degToIn(Rdeg - lastRdeg, VERT_DIAM_IN);
    const double dHraw = SIGN_H  * degToIn(Hdeg - lastHdeg, HORZ_DIAM_IN);

    // =============================
    // Glitch protection thresholds
    // =============================
    // (Tune these if needed. Start conservative.)
    constexpr double MAX_STEP_IN   = 3.0;   // inches per 10ms
    constexpr double MAX_STEP_RAD  = 0.6;   // rad per 10ms (~34 deg)

    // "Soft clamp" caps even non-spike steps (prevents huge pose jumps)
    constexpr double CLAMP_STEP_IN  = 2.0;  // inches per step allowed into integration
    constexpr double CLAMP_STEP_RAD = 0.25; // rad per step allowed into integration (~14 deg)

    const bool spike =
      (std::fabs(dLraw) > MAX_STEP_IN) ||
      (std::fabs(dRraw) > MAX_STEP_IN) ||
      (std::fabs(dHraw) > MAX_STEP_IN) ||
      (std::fabs(dThetaRaw) > MAX_STEP_RAD);

    // =============================
    // If spike:
    // =============================
    if (spike) {
      // Rate-limit warning spam
      static int warnCooldown = 0;
      if (warnCooldown <= 0) {
        warnCooldown = 250;
        printf("[WARN] ODOM spike DROPPED: dL=%.2f dR=%.2f dH=%.2f dTh=%.3f\n",
               dLraw, dRraw, dHraw, dThetaRaw);
        pros::lcd::print(5, "ODOM SPIKE DROP");
      } else {
        warnCooldown -= loopMs;
      }

      // Re-baseline so the next delta is sane
      lastLdeg = Ldeg;
      lastRdeg = Rdeg;
      lastHdeg = Hdeg;
      lastHeadingRad = headingRad;
      odomTheta = headingRad;

      pros::delay(loopMs);
      continue;
    }

    // =============================
    // update baselines
    // =============================
    lastLdeg = Ldeg;
    lastRdeg = Rdeg;
    lastHdeg = Hdeg;

    lastHeadingRad = headingRad;
    odomTheta = headingRad;

    // =============================
    // Soft clamp deltas
    // =============================
    const double dL = clampd_local(dLraw, -CLAMP_STEP_IN,  CLAMP_STEP_IN);
    const double dR = clampd_local(dRraw, -CLAMP_STEP_IN,  CLAMP_STEP_IN);
    const double dH = clampd_local(dHraw, -CLAMP_STEP_IN,  CLAMP_STEP_IN);
    const double dTheta = clampd_local(dThetaRaw, -CLAMP_STEP_RAD, CLAMP_STEP_RAD);

    // --- Robot-frame translation ---
    const double dy = (dL + dR) * 0.5;
    const double dx = dH - (H_OFFSET_IN * dTheta);

    // --- Robot -> Field ---
    const double c = std::cos(odomTheta);
    const double s = std::sin(odomTheta);

    const double fieldDx = dx * c - dy * s;
    const double fieldDy = dx * s + dy * c;

    odomX += fieldDx;
    odomY += fieldDy;

    // =============================
    // Additional sanity warnings 
    // =============================
    constexpr double TURN_ONLY_RAD = 0.10;
    constexpr double TURN_DX_MAX   = 1.0;
    constexpr double TURN_DY_MAX   = 1.0;

    constexpr double VERT_MISMATCH_IN = 1.0;
    constexpr double IMU_WHEEL_DTHETA_WARN = 0.08;

    static int warnCooldown2 = 0;
    if (warnCooldown2 > 0) warnCooldown2 -= loopMs;

    const bool vertMismatch =
      (std::fabs(dThetaRaw) < 0.03) && (std::fabs(dLraw - dRraw) > VERT_MISMATCH_IN);

    const bool turnDrift =
      (std::fabs(dThetaRaw) > TURN_ONLY_RAD) &&
      (std::fabs(dx) > TURN_DX_MAX || std::fabs(dy) > TURN_DY_MAX);

    double dThetaWheels = 0.0;
    if (TRACK_WIDTH_IN > 0.1) {
      dThetaWheels = (dRraw - dLraw) / TRACK_WIDTH_IN;
    }

    const bool turnDisagree =
      (TRACK_WIDTH_IN > 0.1) &&
      (std::fabs(dThetaRaw) > 0.02) &&
      (std::fabs(dThetaRaw - dThetaWheels) > IMU_WHEEL_DTHETA_WARN);

    if (warnCooldown2 <= 0 && (vertMismatch || turnDrift || turnDisagree)) {
      warnCooldown2 = 250;

      if (vertMismatch) {
        printf("[WARN] Vert mismatch: dL=%.2f dR=%.2f (dL-dR=%.2f)\n", dLraw, dRraw, (dLraw - dRraw));
      }
      if (turnDrift) {
        printf("[WARN] Turn drift: dTh=%.3f dx=%.2f dy=%.2f (check H_OFFSET/slip)\n", dThetaRaw, dx, dy);
      }
      if (turnDisagree) {
        printf("[WARN] Turn disagree: IMU dTh=%.3f wheels dTh=%.3f (TRACK_WIDTH/scale)\n",
               dThetaRaw, dThetaWheels);
      }

      pros::lcd::print(5, "ODOM WARN");
    }

    // --- Brain LCD every ~200ms ---
    // if (++lcdCounter >= 20) {
    //   lcdCounter = 0;
    //   pros::lcd::print(0, "X: %.2f in", odomX);
    //   pros::lcd::print(1, "Y: %.2f in", odomY);
    //   pros::lcd::print(2, "H: %.1f deg", odomTheta * 180.0 / M_PI);
    //   pros::lcd::print(3, "dL:%.2f dR:%.2f", dLraw, dRraw);
    //   pros::lcd::print(4, "dH:%.2f dTh:%.3f", dHraw, dThetaRaw);
    // }

    // --- USB terminal every ~500ms ---
    if (++usbCounter >= 50) {
      usbCounter = 0;
      printf("[ODOM] X=%.2f  Y=%.2f  H=%.1f\n",
             odomX, odomY, odomTheta * 180.0 / M_PI);
    }

    pros::delay(loopMs);
  }
}