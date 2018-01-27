#include "Arduino.h"
#include "Motor_Control.h"

//#define debug

//constructor: initializes all pins and parameters
Motor_Control::Motor_Control(void)
{
}

void Motor_Control::init(uint8_t en, uint8_t in1, uint8_t in2)
{

  _en = en;
  _in1 = in1;
  _in2 = in2;

  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}


void Motor_Control::set_pwr(float pwr)
{
  //set the  left direction
  if (pwr < 0) {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
  }
  else {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
  }

  analogWrite(_en, constrain(abs(pwr), 0 , 255));
}



