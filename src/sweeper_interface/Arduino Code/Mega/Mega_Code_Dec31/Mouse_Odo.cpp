//Adapted from code written by Martijn The -> post [at] martijnthe.nl

#include "Arduino.h"
#include "Mouse_Odo.h"
//define debug 1

//pwr +5v orng-yel
//gnd purp-grn

Mouse_Odo::Mouse_Odo(int pSDIO, int pSCLK) : mOptical1(pSCLK, pSDIO)
{
}

void Mouse_Odo::start()
{

#ifdef debug
  Serial.begin(115200);
  Serial.print("Initializing Mouse...");
#endif

  mOptical1.begin();
  //default mouse gains:
  mMouseGainTheta = 1;
  mMouseGainXY = 1;

#ifdef debug
  Serial.println("Done");
#endif
}

void Mouse_Odo::set_mouse_gains(float nMouseGainTheta, float nMouseGainXY)
{
#ifdef debug
  Serial.print("Setting Gains...");
#endif
  mMouseGainTheta = nMouseGainTheta;
  mMouseGainXY = nMouseGainXY;

#ifdef debug
  Serial.println("Done");
#endif
}

void Mouse_Odo::update_mouse(double pMousePose[3])
{

  static const  float max_delta_x = MAX_VEL_THETA * DISTANCE_FROM_WHEELS * DELTA_T;
  static const float max_delta_y = MAX_VEL_X * DELTA_T;

#ifdef debug
  Serial.print("Update...");
#endif


  double i =  -(double)mOptical1.dx() * 0.0635;                 // Read the dX register and in/decrease X with that value resolution is 400 cpi --> 25.4 mm/in / 400 c/in  =.0635 c/mm
  double j = (double)mOptical1.dy() * 0.0635;                 // Same thing for dY register.....

  //transform the incremental change in i,j to x,y,theta
  //only update mouse position if j is a reasonable number
  if (j < max_delta_y) {
    pMousePose[0] += j * cos(pMousePose[2]) * mMouseGainXY; //x
    pMousePose[1] += j * sin(pMousePose[2]) * mMouseGainXY; //y
  }
  if (i < max_delta_x) {
    pMousePose [2] += i / DISTANCE_FROM_WHEELS * mMouseGainTheta; //theta: derived from robot dimensions (100 mm) and arc length = r*theta
  }

#ifdef debug
  Serial.print("Done: dx = ");
  Serial.print(i);
  Serial.print(", dy: ");
  Serial.println(j);
#endif
}

