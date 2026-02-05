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

// Rotation sensor returns degrees. Convert deg -> inches for wheel travel.
static inline double degToIn(double deg, double wheelDiamIn) {
  return (deg / 360.0) * (M_PI * wheelDiamIn);
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
  odomX = xIn;
  odomY = yIn;

  LVerticalTracker.reset_position();
  RVerticalTracker.reset_position();
  HorizontalTracker.reset_position();

  IMU.tare_rotation();
  odomTheta = 0.0;
}

void odomTask() {
  int lcdCounter = 0;
  int usbCounter = 0;

  // Wait for IMU calibration
  while (IMU.is_calibrating()) pros::delay(10);

  double lastLdeg = LVerticalTracker.get_position();
  double lastRdeg = RVerticalTracker.get_position();
  double lastHdeg = HorizontalTracker.get_position();

  double lastHeadingRad = wrapPi(degToRad(IMU.get_rotation()));
  odomTheta = lastHeadingRad;

  const int loopMs = 10;

  while (true) {
    // --- Wheel positions (deg) ---
    const double Ldeg = LVerticalTracker.get_position();
    const double Rdeg = RVerticalTracker.get_position();
    const double Hdeg = HorizontalTracker.get_position();

    // --- Delta inches (apply sign) ---
    const double dL = SIGN_LV * degToIn(Ldeg - lastLdeg, VERT_DIAM_IN);
    const double dR = SIGN_RV * degToIn(Rdeg - lastRdeg, VERT_DIAM_IN);
    const double dH = SIGN_H  * degToIn(Hdeg - lastHdeg, HORZ_DIAM_IN);

    lastLdeg = Ldeg;
    lastRdeg = Rdeg;
    lastHdeg = Hdeg;

    // --- Heading from IMU ---
    double headingRad = wrapPi(degToRad(IMU.get_rotation()));
    const double dTheta = wrapPi(headingRad - lastHeadingRad);
    lastHeadingRad = headingRad;

    odomTheta = headingRad;

    // --- Robot-frame translation ---
    // Forward from average vertical wheels (cancels rotation)
    const double dy = (dL + dR) * 0.5;

    // Strafe corrected for rotation-induced motion on horizontal tracker
    const double dx = dH - (H_OFFSET_IN * dTheta);

    // --- Robot -> Field ---
    const double c = std::cos(odomTheta);
    const double s = std::sin(odomTheta);

    const double fieldDx = dx * c - dy * s;
    const double fieldDy = dx * s + dy * c;

    odomX += fieldDx;
    odomY += fieldDy;

    // =============================
    // Slip / sanity checks + warnings
    // =============================
    // Tune these thresholds to your robot
    constexpr double MAX_STEP_IN   = 3.0;   // inches per 10ms -> likely glitch
    constexpr double MAX_STEP_RAD  = 0.6;   // rad per 10ms (~34 deg) -> likely glitch

    constexpr double TURN_ONLY_RAD = 0.10;  // rad per step: "we are turning"
    constexpr double TURN_DX_MAX   = 1.0;   // allowed dx while turning (in/step)
    constexpr double TURN_DY_MAX   = 1.0;   // allowed dy while turning (in/step)

    constexpr double VERT_MISMATCH_IN = 1.0;     // |dL - dR| too big when not turning
    constexpr double IMU_WHEEL_DTHETA_WARN = 0.08; // rad disagreement per step (~4.6 deg)

    // Rate-limit warnings so you don't spam
    static int warnCooldown = 0;
    if (warnCooldown > 0) warnCooldown -= loopMs;

    const bool spike =
      (std::fabs(dL) > MAX_STEP_IN) ||
      (std::fabs(dR) > MAX_STEP_IN) ||
      (std::fabs(dH) > MAX_STEP_IN) ||
      (std::fabs(dTheta) > MAX_STEP_RAD);

    const bool vertMismatch =
      (std::fabs(dTheta) < 0.03) && (std::fabs(dL - dR) > VERT_MISMATCH_IN);

    const bool turnDrift =
      (std::fabs(dTheta) > TURN_ONLY_RAD) &&
      (std::fabs(dx) > TURN_DX_MAX || std::fabs(dy) > TURN_DY_MAX);

    // Optional: compare wheel-based turn vs IMU turn (sanity check TRACK_WIDTH_IN)
    double dThetaWheels = 0.0;
    if (TRACK_WIDTH_IN > 0.1) {
      dThetaWheels = (dR - dL) / TRACK_WIDTH_IN; // approx rad
    }
    const bool turnDisagree =
      (TRACK_WIDTH_IN > 0.1) &&
      (std::fabs(dTheta) > 0.02) &&
      (std::fabs(dTheta - dThetaWheels) > IMU_WHEEL_DTHETA_WARN);

    if (warnCooldown <= 0 && (spike || vertMismatch || turnDrift || turnDisagree)) {
      warnCooldown = 250; // ms between warning bursts

      if (spike) {
        printf("[WARN] ODOM spike: dL=%.2f dR=%.2f dH=%.2f dTh=%.3f\n", dL, dR, dH, dTheta);
      }
      if (vertMismatch) {
        printf("[WARN] Vert mismatch: dL=%.2f dR=%.2f (dL-dR=%.2f)\n", dL, dR, (dL - dR));
      }
      if (turnDrift) {
        printf("[WARN] Turn drift: dTh=%.3f dx=%.2f dy=%.2f (check H_OFFSET/slip)\n", dTheta, dx, dy);
      }
      if (turnDisagree) {
        printf("[WARN] Turn disagree: IMU dTh=%.3f wheels dTh=%.3f (TRACK_WIDTH/scale)\n",
               dTheta, dThetaWheels);
      }

      // Optional short indicator on brain LCD line 5
      pros::lcd::print(5, "ODOM WARN");
    }

    // --- Brain LCD every ~200ms ---
    if (++lcdCounter >= 20) {   // 20 * 10ms = 200ms
      lcdCounter = 0;
      pros::lcd::print(0, "X: %.2f in", odomX);
      pros::lcd::print(1, "Y: %.2f in", odomY);
      pros::lcd::print(2, "H: %.1f deg", odomTheta * 180.0 / M_PI);
    }

    // --- USB terminal every ~500ms ---
    if (++usbCounter >= 50) {   // 50 * 10ms = 500ms
      usbCounter = 0;
      printf("[ODOM] X=%.2f  Y=%.2f  H=%.1f\n",
             odomX, odomY, odomTheta * 180.0 / M_PI);
    }

    pros::delay(loopMs);
  }
}

