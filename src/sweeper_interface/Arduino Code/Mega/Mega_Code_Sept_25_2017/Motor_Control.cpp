#include "Arduino.h"
#include "Motor_Control.h"

//#define debug

//constructor: initializes all pins and parameters
Motor_Control::Motor_Control(void)
  : mPID_R(&mCurSPD_R, &mCurPWR_R, &mSetSPD_R, KP_L, KI_L, KD_L, DIRECT)
  , mPID_L(&mCurSPD_L, &mCurPWR_L, &mSetSPD_L, KP_L, KI_L, KD_L, DIRECT)
{

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFTENC, INPUT);
  pinMode(RIGHTENC, INPUT);

  //initialize PID variables
  mSetSPD_R = 0;
  mSetSPD_L = 0;
  mCurPWR_R = 0;
  mCurPWR_L = 0;
  mLastime_L = 0;
  mLastime_R = 0;
  mlastAccel = 0;

  //set power output limits
  mPID_L.SetOutputLimits(-255, 255);
  mPID_R.SetOutputLimits(-255, 255);

  //set sample time
  mPID_L.SetSampleTime(50);
  mPID_R.SetSampleTime(50);

  //enable PID
  mPID_L.SetMode(AUTOMATIC);
  mPID_R.SetMode(AUTOMATIC);

}

void Motor_Control::set_tar_spd(double lspd, double rspd)
{
  mTarSPD_L = lspd;
  mTarSPD_R = rspd;

}

void Motor_Control::set_motor_spd()
{

  //increment PID input speeds towards target speeds
  double inc = ACC_LIM * (micros() - mlastAccel) / 1000000.0;
  mSetSPD_L += constrain(mTarSPD_L - mSetSPD_L, -inc, inc);
  mSetSPD_R += constrain(mTarSPD_R - mSetSPD_R, -inc, inc);
  mlastAccel = micros();

  //set the left side direction
  if (mSetSPD_L) {
    mSign_L = mSetSPD_L / abs(mSetSPD_L);
  } else mSign_L = 0;


  //set right side direction
  if (mSetSPD_R) {
    mSign_R = mSetSPD_R / abs(mSetSPD_R);
  } else mSign_R = 0;



  //set the  left direction
  if (mSetSPD_L < 0) {
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  }
  else {
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
  }

  //set the right direction
  if (mSetSPD_R < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }


}

void Motor_Control::update_pose(float pose[3])
{
  //D*pi*224

  float d_right = (float)mCount_R * MM_PER_TICK; //pi*60mm/224
  float d_left = (float)mCount_L * MM_PER_TICK;
  float d_center = ((d_right + d_left) / 2) * ENC_GAIN_X;


  //right hand rule; ccw is +, cw is -
  //x'= x + d_ctr*cos(theta)
  pose[0] += d_center * cos(pose[2]);

  //y'
  pose[1] += d_center * sin(pose[2]);

  //theta'd_baseline
  pose[2] += ((d_right - d_left) / D_WHEELBASE) * ENC_GAIN_THETA;

  //make sure theta is within 0-> 2PI rad
  if (pose[2] >= 6.283185307)
  {
    pose[2] -= 6.283185307;
  } else if (pose[2] <= -6.283185307)
  {
    pose[2] += 6.283185307;
  }

  //zero counts until next update
  mCount_L = 0;
  mCount_R = 0;
}

void Motor_Control::update_PID()
{

  //adaptive PID values; aggressive constants used above 30 mm/s
  if (abs(mSetSPD_R) < 30)
  {
    mPID_R.SetTunings(KP_L, KI_L, KD_L);
  } else
  {
    mPID_R.SetTunings(KP_H, KI_H, KD_H);
  }
  if (abs(mSetSPD_L) < 30)
  {
    mPID_L.SetTunings(KP_L, KI_L, KD_L);
  } else
  {
    mPID_L.SetTunings(KP_H, KI_H, KD_H);
  }

  double e;
  //RIGHT PID
  if (mSetSPD_R == 0) {
    mCurPWR_R = 0;
  }
  else {
    if (( micros() - mLastime_R) > 2244000 / abs(mSetSPD_R)) mCurSPD_R = 0.0; // stop motors from stalling at slow speeds; if this period > 3*expected period
    mPID_R.Compute();
  }


  //LEFT PID
  if (mSetSPD_L == 0) {
    mCurPWR_L = 0;
  }
  else {
    if (micros() - mLastime_L > 2244000 / abs( mSetSPD_L)) mCurSPD_L = 0.0;
    mPID_L.Compute();
  }

  analogWrite(ENA, abs(mCurPWR_R));
  analogWrite(ENB, abs(mCurPWR_L));

#ifdef debug
  Serial.print(mSetSPD_L);
  Serial.print(", ");
  Serial.println(mCurSPD_L);

#endif

  //increment motor speeds
  set_motor_spd();
}

void Motor_Control::service_left_enc()
{
  //estimate current ground speed in mm/s
  noInterrupts();
  long temp;
  long mCurtime_L = micros();

  temp = mCurtime_L - mLastime_L;
  if (temp > 1000) {
    mCurSPD_L = (double)mSign_L * MM_PER_TICK * 1000000 / (double) temp; //speed in mm/s
    mLastime_L = mCurtime_L;
    mCount_L += mSign_L;

  }
  interrupts();
}

void Motor_Control::service_right_enc()
{
  //estimate current ground speed in mm/s
  noInterrupts();
  long temp;
  long mCurtime_R = micros();

  temp = mCurtime_R - mLastime_R;
  if (temp > 1000) {
    mCurSPD_R = mSign_R * MM_PER_TICK * 1000000 / (double) temp;
    mLastime_R = mCurtime_R;
    mCount_R += mSign_R;

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

