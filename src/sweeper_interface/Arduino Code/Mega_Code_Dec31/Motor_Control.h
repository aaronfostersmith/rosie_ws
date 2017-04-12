/*
  Motor_Control.h - Library for controlling sweeper motors and updating encoder pose.
  Created by Aaron Foster Smith, Dec 17, 2016.
*/

#include "Arduino.h"

#ifndef Motor_Control_h
#define Motor_Control_h



class Motor_Control
{

    //L298 Control Pins
#define ENA 44 //pwm grn
#define ENB 45 //pwm blue
#define IN1 30 //digital wh
#define IN2 31 //digital red
#define IN3 32 //digital orng 
#define IN4 33 //digital yel

    //encoder pins
#define LEFTENC 18 //ext int
#define RIGHTENC 19 //ext int

    //motor definitions
#define LEFTMTR 0
#define RIGHTMTR 1

  public:
    Motor_Control(void); //constructor
    int get_left_enc(void);
    int get_right_enc();
    void set_PID_gains(float P_L, float I_L, float D_L, float P_R, float I_R, float D_R); //set our PID values
    void set_enc_gains(float gainTheta, float gainXY); //set the encoder gains
    void set_motor_spd(bool mtr, double spd);
    void update_pose(float pose[3]);
    void update_PID();
    void service_left_enc();
    void service_right_enc();


  private:
    double mKp_L, mKi_L, mKd_L, mKp_R, mKi_R, mKd_R;
    double mGainTheta, mGainXY;
    double mSetSPD_R, mCurSPD_R, mI_R, mPre_e_R ;
    double mSetSPD_L, mCurSPD_L, mI_L, mPre_e_L ; // desired rpm, ticks per second (setpoint),current ticks per second (input)
    double mCurPWR_L, mCurPWR_R; //current motor power (output)
    long mLastime_L, mLastime_R; //encoder time tracking variables
    int mSign_L, mCount_L, mSign_R, mCount_R;
};

#endif
