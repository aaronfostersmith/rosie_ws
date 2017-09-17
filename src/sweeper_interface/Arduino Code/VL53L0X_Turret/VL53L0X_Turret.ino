/*WIRING:
  TCRT5000:
  - Black - Gnd
  - Green - Detector
  - Yellow - Emitter

  VL53L0X:
  - Red - Vin
  - Black - Gnd
  - Yellow - SCL - A5
  - Orange - SDA - A4
  - Brown - GPIO1 (UNUSED)
  - Green - XSHUT (UNUSED)
*/
#include <PID_v1.h>

#include <ros.h>
//a header file for each msg type must be included
#include <sweeper_interface/laser_range.h>
#include <Wire.h>
#include <VL53L0X.h>

#define MOTOR_PIN 5
#define ENC_PIN 3
#define INDEX_PIN 2
#define ENCODER_TICKS 12

double Setpoint, Input, Output;
PID drivePID(&Input, &Output, &Setpoint, 15, 130, 5, DIRECT);


VL53L0X laser;
//#define LONG_RANGE


#define PI 3.14159265359

float ANGLE_MIN = -PI; //PI/2 is straight forward
float ANGLE_MAX = PI;
int TIME_INCREMENT = 30000; // us WHY CAN"T GO OVER 30000? OVERFLOW?
int N;

#define RANGE_MIN 0.02 //meters
#define RANGE_MAX 2.00 //meters


//instantiate a handle for the node
#define ROSSERIAL // ros connectivity
#ifdef ROSSERIAL
ros::NodeHandle n;
//publisher
sweeper_interface::laser_range scan_msg;
ros::Publisher scan_pub("/sweeper/rawLaser", &scan_msg);
#endif

void setup() {

  pinMode(ENC_PIN, INPUT_PULLUP);
  pinMode(INDEX_PIN, INPUT_PULLUP);

  pinMode(MOTOR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), ENC_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(INDEX_PIN), IND_ISR, RISING);



  //initialize Rosserial
#ifdef ROSSERIAL
  initComms();
#endif

  //init laser and I2C
  Wire.begin();
  laser.init();
  laser.setTimeout(25);


#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  laser.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

  // set timing budget to
  laser.setMeasurementTimingBudget(25000);


  //calculate some paramters
  Setpoint = 1.0 / ((float)TIME_INCREMENT * N / 1000000); //Hz
  Input = 0;

  drivePID.SetOutputLimits(16, 255);
  drivePID.SetSampleTime(1000 / (Setpoint * ENCODER_TICKS));
  drivePID.SetMode(AUTOMATIC);


}

long lastenc = 0;
void ENC_ISR() {
  long thistime = micros();
  if(thistime - lastenc >50){
  Input = 1000000.0 / (ENCODER_TICKS * (thistime - lastenc));
  lastenc = thistime;
  }
}

long lastind = 0;
void IND_ISR() {
  long thistime = millis();
  
  if (thistime - lastind >100) {
    scan_msg.seq = 0;
    lastind = thistime;
  }
}

//initialize ros nodes and advertise Publishers, subcribe to topics
#ifdef ROSSERIAL
void initComms() {
  n.getHardware() -> setBaud(57600);
  n.initNode();
  ros::Time begin = n.now();

  //advertise publishers
  n.advertise(scan_pub);

  while (!n.connected()) {
    n.spinOnce();
  }


  //get parameters
  //  if (! n.getParam("~angle_min", &ANGLE_MIN)) {
  ANGLE_MIN = -PI;
  //  }
  //
  //  if (! n.getParam("~angle_max", &ANGLE_MAX)) {
  ANGLE_MAX = PI;
  //  }
  //

  if (! n.getParam("~num_pts", &N)) {
    N = 48;
  }
}
#endif

//get and publish a single scan
void pub_ping() {
  unsigned long pingStart;
  pingStart = micros();

  //get a reading
  int range = laser.readRangeSingleMillimeters();
  if (laser.timeoutOccurred() || range > RANGE_MAX * 1000 || range < RANGE_MIN * 1000) {
    range = RANGE_MAX * 1000 + 1;
  }

#ifdef ROSSERIAL
  scan_msg.range = range;
  scan_msg.seq ++;
  if (scan_msg.seq >= N) scan_msg.seq = 0;
  //publish the scan data
  scan_pub.publish(&scan_msg); //mousePose_msg is published to topic mouse_odo
  n.spinOnce();
#endif

  //delay until TIME_INCREMENT is up
  while (micros() - pingStart < TIME_INCREMENT) {}
}

//main loop
void loop() {

  pub_ping();
  drivePID.Compute();
  analogWrite(MOTOR_PIN, Output);
#ifdef ROSSERIAL
  n.spinOnce();
#endif

}
