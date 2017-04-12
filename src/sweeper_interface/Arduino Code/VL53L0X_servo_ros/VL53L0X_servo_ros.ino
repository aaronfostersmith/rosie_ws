#include <Servo.h>
#include <ros.h>
//a header file for each msg type must be included
#include <std_msgs/Int16.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X laser;
#define LONG_RANGE

Servo laserServo;
/*
  This program publishes a LaserScan msg with four points while rotating the turret
*/

#define ANGLE_MIN 0 //PI/2 is straight forward
#define ANGLE_MAX 3.14159265359
#define ANGLE_INCREMENT 0.0245436926 //rad == 4 deg
#define TIME_INCREMENT 30000 // us
long SCAN_TIME; // us
int N;
#define RANGE_MIN 0.03 //meters
#define RANGE_MAX 2.00 //meters
#define PI 3.14159265359
#define RETURN_TIME 1000000 //us

#define SERVO_PIN 3

//#define SERIALDEBUG;

//instantiate a handle for the node
#define ROSSERIAL // ros connectivity
#ifdef ROSSERIAL
ros::NodeHandle n;
//publisher
std_msgs::Int16 scan_msg;
ros::Publisher scan_pub("/sweeper/rawLaser", &scan_msg);
#endif

void setup() {
  N = (ANGLE_MAX - ANGLE_MIN) / (ANGLE_INCREMENT);
  SCAN_TIME = (long)N * TIME_INCREMENT + RETURN_TIME;


#ifdef SERIALDEBUG
  Serial.begin(115200);
  Serial.print("Serial Debug Enabled: Expecting ");
  Serial.print(N);
  Serial.print(" datapoints every ");
  Serial.print(SCAN_TIME);
  Serial.println(" seconds.");
  delay(3000);
#endif

  //initialize Rosserial
#ifdef ROSSERIAL
  initComms();
#endif


  //init laser and I2C
  Wire.begin();

  laser.init();
  laser.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  laser.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

  // reduce timing budget to 20 ms (default is about 33 ms)
  laser.setMeasurementTimingBudget(TIME_INCREMENT - 10000);

  //init servo
  laserServo.attach(SERVO_PIN);

#ifdef SERIALDEBUG
  Serial.println("Initialized");
#endif
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

}
#endif

void get_scan() {
  unsigned long pingStart;

  for (int i = 0; i < N; i++) {
    pingStart = micros();
    //get a reading and advance the servo
    int range = laser.readRangeSingleMillimeters();
    if (laser.timeoutOccurred() || range > RANGE_MAX * 1000) {
      range = RANGE_MAX + 1;
    }
#ifdef ROSSERIAL
    scan_msg.data = range;
    //publish the scan data
  scan_pub.publish(&scan_msg); //mousePose_msg is published to topic mouse_odo
  n.spinOnce();
#endif
#ifdef SERIALDEBUG
    //  Serial.println(range);
#endif

    //delay until TIME_INCREMENT is up
    laserServo.write((int)((ANGLE_MIN + i * ANGLE_INCREMENT) * 180 / PI));
    while (micros() - pingStart < TIME_INCREMENT) {}
  }
  laserServo.write((int)(ANGLE_MIN * 180 / PI));
}

//main loop
void loop() {

  unsigned long scanStart = micros();

  get_scan();

//#ifdef ROSSERIAL
//  //publish the scan data
//  scan_pub.publish(&scan_msg); //mousePose_msg is published to topic mouse_odo
//  n.spinOnce();
//#endif
  //spin until the servo returns for another sweep

  while (micros() - scanStart < SCAN_TIME)  {
#ifdef ROSSERIAL
    n.spinOnce();
#endif
  }

}
