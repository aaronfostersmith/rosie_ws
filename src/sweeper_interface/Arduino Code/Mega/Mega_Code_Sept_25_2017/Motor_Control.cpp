#include "Arduino.h"
#include "Motor_Control.h"



//constructor: initializes all pins and parameters
Motor_Control::Motor_Control(void)
{
  //default values for gains and PID constants
  mKp_L = 0.4;
  mKi_L = 0.0003;
  mKd_L = 1.25;
  mKp_R = 0.4;
  mKi_R = 0.0003;
  mKd_R = 1.25;

  mGainTheta = 1;
  mGainXY = 1;

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFTENC, INPUT);
  pinMode(RIGHTENC, INPUT);
}

void Motor_Control::set_PID_gains(float pP_L, float pI_L, float pD_L, float pP_R, float pI_R, float pD_R)
{
  mKp_R = pP_R;
  mKi_R = pI_R;
  mKd_R = pD_R;
  mKp_L = pP_L;
  mKi_L = pI_L;
  mKd_L = pD_L;
}

void Motor_Control::set_enc_gains(float pGainTheta, float pGainXY)
{
  mGainTheta = pGainTheta;
  mGainXY = pGainXY;
}


void Motor_Control::set_motor_spd(bool mtr, double spd)
{
  int dir1, dir2, en;

  if (abs(spd) < 10.0) spd = 0; //10mm/s min


  //Select left or right side pins
  noInterrupts();
  switch (mtr) {
    case RIGHTMTR:
      dir1 = IN1;
      dir2 = IN2;
      en = ENB;
      mSetSPD_R = abs(spd);
      mI_R = 0;
      mSign_R = spd / mSetSPD_R;
      break;
    case LEFTMTR:
      dir1 = IN4;
      dir2 = IN3;
      en = ENA;
      mSetSPD_L = abs(spd);
      mI_L = 0;
      mSign_L = spd / mSetSPD_L;
      break;
    default:
      return;
      break;
  }
  //set the direction
  if (spd < 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  }
  else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  }

  interrupts();
}

void Motor_Control::update_pose(float pose[3])
{
  //D*pi*224

  float d_right = (float)mCount_R * 0.8415 * mSign_R; //pi*60mm/224
  float d_left = (float)mCount_L * 0.8415 * mSign_L;
  float d_center = ((d_right + d_left) / 2) * mGainXY;

  //right hand rule; ccw is +, cw is -
  //x'= x + d_ctr*cos(theta)
  pose[0] += d_center * cos(pose[2]);

  //y'
  pose[1] += d_center * sin(pose[2]);

  //theta'd_baseline = 190 mm
  pose[2] += ((d_right - d_left) / 190) * mGainTheta;
  //zero counts until next update

  mCount_L = 0;
  mCount_R = 0;
}

void Motor_Control::update_PID()
{
  noInterrupts();
  double e;
  //RIGHT PID
  if (mSetSPD_R == 0) {
    mCurPWR_R = 0;
  }
  else {
    if (( micros() - mLastime_R) > 2244000 / mSetSPD_R) mCurSPD_R = 0.0; // stop motors from stalling at slow speeds; if this period > 3*expected period
    e = mSetSPD_R - mCurSPD_R; //calculate error
    mI_R += e; //calculate integral
    mCurPWR_R += mKp_R * e + mKi_R * mI_R + mKd_R * (e - mPre_e_R); //PID calculation
    mCurPWR_R = constrain(mCurPWR_R, 0, 255); //stop the power from going out of range
    mPre_e_R = e; //save error for derivative calculation next time
  }


  //LEFT PID
  if (mSetSPD_L == 0) {
    mCurPWR_L = 0;
  }
  else {
    if (micros() - mLastime_L > 2244000 / mSetSPD_L) mCurSPD_L = 0.0;
    e = mSetSPD_L - mCurSPD_L;
    mI_L += e;
    mCurPWR_L += mKp_L * e + mKi_L * mI_L + mKd_L * (e - mPre_e_L ) ;
    mCurPWR_L = constrain(mCurPWR_L, 0, 255);
    mPre_e_L = e;
     }

    analogWrite(ENA, mCurPWR_R);
    analogWrite(ENB, mCurPWR_L);

    interrupts();
  }

  void Motor_Control::service_left_enc()
  {
    //estimate current ticks per second
    noInterrupts();
    long temp;
    long mCurtime_L = micros();

    temp = mCurtime_L - mLastime_L;
    if (temp > 1000) {
      mCurSPD_L = 772522.78 / (double) temp; // 0.77252278367 [mm/tick]/ temp [s/tick] = speed in mm/s
      mLastime_L = mCurtime_L;
      mCount_L++;

    }
    interrupts();
  }

  void Motor_Control::service_right_enc()
  {
    //estimate current ticks per second
    noInterrupts();
    long temp;
    long mCurtime_R = micros();

    temp = mCurtime_R - mLastime_R;
    if (temp > 1000) {
      mCurSPD_R = 772522.78 / (double)temp;
      mLastime_R = mCurtime_R;
      mCount_R ++;

    }
    interrupts();
  }

  int Motor_Control::get_left_enc()
  {
    return LEFTENC;
  }

    int Motor_Control::get_right_enc()
  {
    return RIGHTENC;
  }


