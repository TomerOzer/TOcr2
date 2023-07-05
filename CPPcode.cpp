#include "Hcode.h"
#include "Arduino.h"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

M_Robot::M_Robot(int _m1a, int _m1b, int _m1e, int _m2a, int _m2b, int _m2e, int _m3a, int _m3b, int _m3e, int _m4a, int _m4b, int _m4e)
{
  m1a = _m1a;
  m1b = _m1b;
  m1e = _m1e;
  m2a = _m2a;
  m2b = _m2b;
  m2e = _m2e;
  m3a = _m3a;
  m3b = _m3b;
  m3e = _m3e;
  m4a = _m4a;
  m4b = _m4b;
  m4e = _m4e;

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m1e, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(m2e, OUTPUT);
  pinMode(m3a, OUTPUT);
  pinMode(m3b, OUTPUT);
  pinMode(m3e, OUTPUT);
  pinMode(m4a, OUTPUT);
  pinMode(m4b, OUTPUT);
  pinMode(m4e, OUTPUT);
}

void M_Robot::begin()
{
  mpu.begin();
  calibrateMPU();
}

void M_Robot::moveF(int speed)
{
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  analogWrite(m1e, speed);

  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
  analogWrite(m2e, speed);

  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  analogWrite(m3e, speed);

  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(m4e, speed);
}

void M_Robot::moveB(int speed)
{
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  analogWrite(m1e, speed);

  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  analogWrite(m2e, speed);

  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  analogWrite(m3e, speed);

  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(m4e, speed);
}

void M_Robot::moveL(int speed)
{
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  analogWrite(m1e, speed);

  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  analogWrite(m2e, speed);

  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  analogWrite(m3e, speed);

  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(m4e, speed);
}

void M_Robot::moveR(int speed)
{
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  analogWrite(m1e, speed);

  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
  analogWrite(m2e, speed);

  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  analogWrite(m3e, speed);

  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(m4e, speed);
}

void M_Robot::turnR(int speed)
{
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  analogWrite(m1e, speed);

  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
  analogWrite(m2e, speed);

  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  analogWrite(m3e, speed);

  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(m4e, speed);
}

void M_Robot::turnL(int speed)
{
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  analogWrite(m1e, speed);

  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  analogWrite(m2e, speed);

  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  analogWrite(m3e, speed);

  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(m4e, speed);
  
}

void M_Robot::calibrateMPU()
{
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) { } // Stop everything if could not connect to MPU6050.

  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true, false); // Gyro and accelerometer.
  Serial.println("Done!\n");
}

void M_Robot::printYaw()
{
  Serial.print("Yaw: ");
  Serial.println(getYaw());
}

float M_Robot::getYaw()
{
  mpu.update();
  int yaw = mpu.getAngleZ() * -1 ;
  if (yaw > 360) {
    yaw -= 360;
  }

  return yaw;
}

void M_Robot::PID_mecanum_set(double _kp, double _ki, double _kd)
{
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

double M_Robot::calc(double sp, double pv)
{
  double error = sp - pv;
  double P = kp * error;

  integral += error;
  double I = ki * integral;

  double derivative = error - pv;
  double D = kd * derivative;

  double output = P + I + D;

  pv = error;

  return output;
}

