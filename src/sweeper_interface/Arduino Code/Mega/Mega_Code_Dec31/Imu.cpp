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

#ifdef debug
  Serial.begin(115200);
  Serial.print("Init Start...");
#endif
  //turn on the accelerometer
  acc.powerOn();
  //set accelerometer range to +- 4g
  acc.setRangeSetting(4);
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

  //initialize offsets
  mOffsets[0] = 0;
  mOffsets[1] = 0;
  mOffsets[2] = 0;
  mOffsets[3] = 0;
  mOffsets[4] = 0;
  mOffsets[5] = 0;
#ifdef debug
  Serial.println("done");
#endif
}


void Imu::Calibrate()
{

#ifdef debug
  Serial.print("Calibrating...");
#endif

  int accelRaw[3]; //temp container for readings
  float readings[3]; //temp container for current readings

  for (int i = 0; i <= 20; i++) {
    //get a new reading from the accelerometer
    acc.readAccel(&accelRaw[0], &accelRaw[1], &accelRaw[2]);
    delay(5);
    //avg the results
    mOffsets[3] += accelRaw[0];
    mOffsets[4] += accelRaw[1];
    mOffsets[5] += accelRaw[2];
  }
  mOffsets[3] /= 20;
  mOffsets[4] /= 20;
  mOffsets[5] /= 20;

  for (int i = 0; i < 20; i++) {
    gyr.readGyro(&readings[0], &readings[1], &readings[2]);
    delay(5);
    mOffsets[0] += readings[0];
    mOffsets[1] += readings[1];
    mOffsets[2] += readings[2];
  }
  
  mOffsets[0] /= 20;
  mOffsets[1] /= 20;
  mOffsets[2] /= 20;

#ifdef debug
  Serial.print("done with offsets: A:");
  for (int i = 3; i < 6; i ++) {
    Serial.print(mOffsets[i]);
    Serial.print(", ");
  }
  Serial.print("G: ");
  for (int i = 0; i < 3; i ++) {
    Serial.print(mOffsets[i]);
    Serial.print(", ");
  }
  Serial.println();
#endif
}



void Imu::update_imu(float imuBytes[9])
{

#ifdef debug
  Serial.print("Updating...");
#endif

  //pull raw Magnetometer data
  MagnetometerRaw raw = mag.ReadRawAxis();
  imuBytes[0] = raw.XAxis;
  imuBytes[1] = raw.YAxis;
  imuBytes[2] = raw.ZAxis;

  // pull raw gyro data and subtract offset

  gyr.readGyro(&imuBytes[3], &imuBytes[4], &imuBytes[5]);
  imuBytes[3] = imuBytes[3] - mOffsets[0];
  imuBytes[4] = imuBytes[4] - mOffsets[1];
  imuBytes[5] = imuBytes[5] - mOffsets[2];

  // pull raw accelerometer data and subtract offset
  int accelRaw[3];
  acc.readAccel(&accelRaw[0], &accelRaw[1], &accelRaw[2]); //read the accelerometer values and store them in variables  x,y,z

  //scale accelerometer data to m/s^2. 10 bit res : scale = 8/2^10*9.81 [m/s^2]=0.0076641
  imuBytes[6] = (accelRaw[0]  - mOffsets[3]) * 0.076641;
  imuBytes[7] = (accelRaw[1] - mOffsets[4]) * 0.076641;
  imuBytes[8] = (accelRaw[2]  - mOffsets[5]) * 0.076641+ 9.81;

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

#ifdef debug
  Serial.print("Done: ");
  for (int i = 0; i < 9; i++) {
    Serial.print(imuBytes[i]);
    Serial.print(", ");
  }
  Serial.println();
#endif
}

