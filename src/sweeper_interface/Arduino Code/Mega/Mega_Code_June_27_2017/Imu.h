/*
  Imu.h - Library for controlling ADNS2610 based odometry sensor.
  Created by Aaron Foster Smith, Dec 17, 2016.
*/
#include <Wire.h> //I2C Library
#include <ADXL345.h>  // ADXL345 Accelerometer Library
#include <HMC5883L.h> // HMC5883L Magnetometer Library
#include <ITG3200.h>  // ITG3200 Gyro Library
#include <Filters.h> //filter library


#ifndef Imu_h
#define Imu_h

class Imu
{
  public:
    Imu();
    Imu(float filtCutFreq);
    void Calibrate();
    bool isLpfOn;
    void update_imu(float imuBytes[9]);
    
  private:
    float mOffsets[6]; //calibration offsets ,gX,gY,Gz,aX,aY,aZ
    float mReadings[9]; // gx,gy,gz,ax,ay,az,mx,my,mz
    FilterOnePole mGxLPF, mGyLPF, mGzLPF, mAxLPF, mAyLPF, mAzLPF, mMxLPF, mMyLPF, mMzLPF;
    ADXL345 acc;
    HMC5883L mag;
    ITG3200 gyr;
};

#endif
