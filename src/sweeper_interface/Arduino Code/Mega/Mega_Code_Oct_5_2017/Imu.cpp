#include "Arduino.h"
#include "Imu.h"
//#define debug 1

//default constructor
Imu::Imu()
{
}

//constructor
Imu::Imu(float filtCutFreq)
  : gyr()
  , mGxLPF(LOWPASS, filtCutFreq) //lpf for gyro x
  , mGyLPF(LOWPASS, filtCutFreq)//lpf for gyro y
  , mGzLPF(LOWPASS, filtCutFreq)//lpf for gyro z
  , acc()
  , mAxLPF(LOWPASS, filtCutFreq)//lpf for accel x
  , mAyLPF(LOWPASS, filtCutFreq)//lpf for accel y
  , mAzLPF(LOWPASS, filtCutFreq)//lpf for accel z
  , mag()
  , mMxLPF(LOWPASS, filtCutFreq)//lpf for mag x
  , mMyLPF(LOWPASS, filtCutFreq)//lpf for mag y
  , mMzLPF(LOWPASS, filtCutFreq)//lpf for mag z

{

  //turn on the accelerometer
  acc.powerOn();
  //set accelerometer range
  acc.setRangeSetting(8);
  //set hardware offsets to 0
  acc.setAxisOffset(0, 0, 0);

  //set the gyro to radians
  gyr.setScaleFactor(1.0, 1.0, 1.0, true); //true ==rad
  //initialize the gyro
  gyr.init(ITG3200_ADDR_AD0_LOW);

  // Set the compass scale to +/- 1.3 Ga of the compass (why?)
  mag.SetScale(1.3);
  // Set the measurement mode to Continuous
  mag.SetMeasurementMode(Measurement_Continuous);

}


void Imu::Calibrate()
{

  int accelRaw[3]; //temp container for readings
  int temp[3]={0,0,0}; //temp container for averaging
  
  for (int i = 0; i < 20; i++) {
    //get a new reading from the accelerometer
    acc.readAccel(&accelRaw[0], &accelRaw[1], &accelRaw[2]);
    delay(20);
    //avg the results
    temp[0] += accelRaw[0];
    temp[1] += accelRaw[1];
    temp[2] += accelRaw[2];
  }
  temp[0] /= -20;
  temp[1] /= -20;
  temp[2] /= -20;

  //set the offset registers in ADXL345
  acc.setAxisOffset(temp[0], temp[1], temp[2]);

  //set gyroscope offsets with 20 samples at 20 ms per/sample
  gyr.zeroCalibrate(20, 20);


}



void Imu::update_imu(float imuBytes[9])
{

  //pull raw Magnetometer data
  MagnetometerRaw raw = mag.ReadRawAxis();
  imuBytes[0] = raw.XAxis;
  imuBytes[1] = raw.YAxis;
  imuBytes[2] = raw.ZAxis;

  // pull raw gyro data
  gyr.readGyro(&imuBytes[3], &imuBytes[4], &imuBytes[5]);
  imuBytes[3] = imuBytes[3];
  imuBytes[4] = imuBytes[4];
  imuBytes[5] = imuBytes[5];

  // pull raw accelerometer data
  int accelRaw[3];
  acc.readAccel(&accelRaw[0], &accelRaw[1], &accelRaw[2]); //read the accelerometer values and store them in variables  x,y,z

  //scale accelerometer data to m/s^2. 10 bit res : scale = 8/2^10*9.81 [m/s^2]=0.0076641
  imuBytes[6] = accelRaw[0] * 0.076641;
  imuBytes[7] = accelRaw[1] * 0.076641;
  imuBytes[8] = accelRaw[2] * 0.076641;

  if (isLpfOn) {
    //filter raw data
    imuBytes[0] = mMxLPF.input( imuBytes[0]);
    imuBytes[1] = mMyLPF.input( imuBytes[1]);
    imuBytes[2] = mMzLPF.input( imuBytes[2]);
    imuBytes[3] = mGxLPF.input( imuBytes[3]);
    imuBytes[4] = mGyLPF.input( imuBytes[4]);
    imuBytes[5] = mGzLPF.input( imuBytes[5]);
    imuBytes[6] = mAxLPF.input( imuBytes[6]);
    imuBytes[7] = mAyLPF.input( imuBytes[7]);
    imuBytes[8] = mAzLPF.input( imuBytes[8]);

  }

}

