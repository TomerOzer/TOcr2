#include "Arduino.h"
#ifndef CLICLI_H
#define CLICLI_H
#include "Hcode.h"

 class clicli {

  public:
   clicli(M_Robot &mechanum_move) ;
   void begin();   //must be called from  void setup()
   void run();   //must be called from  void loop()

  private:
   M_Robot &myrobot;
   int number;

 };
#endif 
