#ifndef HCODE_H
#define HCODE_H

class M_Robot {
public:
  M_Robot(int _m1a, int _m1b, int _m1e, int _m2a, int _m2b, int _m2e, int _m3a, int _m3b, int _m3e, int _m4a, int _m4b, int _m4e);
  void begin();
  void moveF(int speed);
  void moveB(int speed);
  void moveL(int speed);
  void moveR(int speed);
  void turnR(int speed);
  void turnL(int speed);
  void calibrateMPU();
  void printYaw();
  float getYaw();
private:
  int m1a, m1b, m1e, m2a, m2b, m2e, m3a, m3b, m3e, m4a, m4b, m4e;
  void wTLf(int speed);
  void wTLb(int speed);
  void wBRf(int speed);
  void wBRb(int speed);
  void wTRf(int speed);
  void wTRb(int speed);
  void wBLf(int speed);
  void wBLb(int speed);
};

#endif
