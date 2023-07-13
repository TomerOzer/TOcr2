#include "Arduino.h"
#include "Hcode.h"
#include "clicli.h"

int M1a = 7;
int M1b = 6;
int M1e = 5;
int M2a = 12;
int M2b = 13;
int M2e = 11;
int M3a = 2;
int M3b = 3;
int M3e = 4;
int M4a = 9;
int M4b = 8;
int M4e = 10;

M_Robot tocr2(M1a, M1b, M1e, M2a, M2b, M2e, M3a, M3b, M3e, M4a, M4b, M4e);
clicli mycli(tocr2);

void setup()
{
  tocr2.begin();
  tocr2.PID_mecanum_set(1, 0.1, 0.1);
  mycli.begin();
}

void loop()
{
  tocr2.moveF(255);
}
