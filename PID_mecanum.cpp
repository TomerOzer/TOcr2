#include "PID_mecanum.h"

PID_mecanum::PID_mecanum(double kp, double ki, double kd, double sampleTime) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  integral = 0.0;
  pv = 0.0;
  this->sampleTime = sampleTime;
}

double PID_mecanum::calc(double sp, double pv, double kp, double ki, double kd) {
  double error = sp - pv;
  integral += error * sampleTime;
  double derivative = (error - this->pv) / sampleTime;

  double output = kp * error + ki * integral + kd * derivative;

  this->pv = pv; 

  return output;
}

