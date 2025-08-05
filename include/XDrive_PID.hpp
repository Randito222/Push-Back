class PID{

    Public:
    double kP, kI, kD;
    double integral;
    double prevError;
    double output;

    PID(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  double calculate(double error) {
    integral += error;
    double derivative = error - prevError;
    prevError = error;
    output = (kP * error) + (kI * integral) + (kD * derivative);
    return output;
  }

  void reset() {
    integral = 0;
    prevError = 0;
  }
    
}

void x_drive_pid_task();