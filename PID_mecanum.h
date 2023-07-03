#ifndef PID_MECANUM_H
#define PID_MECANUM_H

class PID_mecanum {
public:
  PID_mecanum(double kp, double ki, double kd, double sampleTime);
  double calc(double setpoint, double pv);

private:
  double Kp;
  double Ki;
  double Kd;
  double integral;
  double pv;
  double sampleTime;
};

#endif
