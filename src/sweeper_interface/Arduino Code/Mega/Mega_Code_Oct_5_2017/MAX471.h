/*
  MAX471.h - Library for controlling MAX471 based battery sensor.
  Created by Aaron Foster Smith, Dec 17, 2016.
*/


#ifndef MAX471_h
#define MAX471_h

class MAX471
{
  public:
    MAX471(int currentPin, int voltagePin);
    void start();
    float current();
    float voltage();
  private:
    int mCurPin, mVolPin;
   
};

#endif
