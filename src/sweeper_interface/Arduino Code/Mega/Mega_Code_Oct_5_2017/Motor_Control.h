/*
  Motor_Control.h - Library for controlling sweeper motors and updating encoder pose.
  Created by Aaron Foster Smith, Dec 17, 2016.
*/

#include "Arduino.h"
#include "PID_v1.h"


#ifndef Motor_Control_h
#define Motor_Control_h

//physical constants
#define MM_PER_TICK 0.84149803221 // pi*(diameter of wheel)/(# ticks per rotation of wheel)
#define D_WHEELBASE 185 //mm
#define ACC_LIM 50.0 // mm/s^2

//PID values
//high
#define KP_H 5
#define KI_H 15
#define KD_H 0
//low
#define KP_L 1.25
#define KI_L 7
#define KD_L 0

//encoder gains
#define ENC_GAIN_X 1.0
#define ENC_GAIN_THETA 1.0


class Motor_Control
{

    //L298 Control Pins
#define ENA 4 //pwm grn
#define ENB 5 //pwm blue
#define IN1 31 //digital wh
#define IN2 33 //digital red
#define IN3 35 //digital orng 
#define IN4 37 //digital yel

    //encoder pins
#define LEFTENC 18 //ext int
#define RIGHTENC 19 //ext int

    //motor definitions
#define LEFTMTR 0
#define RIGHTMTR 1

  public:
    Motor_Control(void); //constructor
    int get_left_enc(void);
    int get_right_enc(void);
    void set_tar_spd(double lspd, double rspd);
    void get_EncTicks(int ticks[2]);
    void update_PID();
    void service_left_enc();
    void service_right_enc();


  private:
    void set_motor_spd();
    
    double mTarSPD_R, mTarSPD_L; //desired target speed
    double mSetSPD_R, mSetSPD_L; //setpoint
    double mCurPWR_R, mCurPWR_L; //current motor power (output)
    double mCurSPD_R, mCurSPD_L; //current speed of the wheels (input)
    long mLastime_L, mLastime_R, mlastAccel; //encoder time tracking variables for debounce
    int mSign_L, mCount_L, mSign_R, mCount_R; //direction and count variables
    PID mPID_L, mPID_R; //PID objects
};

#endif
