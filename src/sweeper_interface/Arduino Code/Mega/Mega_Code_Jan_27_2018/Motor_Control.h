/*
  Motor_Control.h - Library for controlling sweeper motors and updating encoder pose.
  Created by Aaron Foster Smith, Dec 17, 2016.
*/

#include "Arduino.h"

#ifndef Motor_Control_h
#define Motor_Control_h

class Motor_Control
{

  public:
    Motor_Control(void); //constructor
    void init(uint8_t en, uint8_t in1, uint8_t in2);
    void set_pwr(float pwr);

  private:
    uint8_t _en;
    uint8_t _in1;
    uint8_t _in2;

};

#endif
