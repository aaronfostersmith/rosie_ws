/*
Mouse_Odo.h - Library for controlling ADNS2610 based odometry sensor.
Created by Aaron Foster Smith, Dec 17, 2016.
*/

#include "Arduino.h"
#include <ADNS2610.h>

#ifndef Mouse_Odo_h
#define Mouse_Odo_h

class Mouse_Odo
{

  #define MAX_VEL_X 75//mm/s
  #define MAX_VEL_THETA 1 //rad/s
  #define DELTA_T 0.05 //s
  #define DISTANCE_FROM_WHEELS 110 //mm
  public:
    
    Mouse_Odo(int pSCLK, int pSDIO);
    void start();
    void set_mouse_gains(float pMouseGainTheta, float pMouseGainXY);
    void update_mouse(double pMousePose[3]);
  private:
    ADNS2610 mOptical1;
    float mMouseGainTheta;
    float mMouseGainXY;

};

#endif

