#include <Servo.h>
#include <ros.h>
//a header file for each msg type must be included
#include <sweeper_interface/laser_range.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X laser;
#define LONG_RANGE

Servo laserServo;
/*
  This program publishes a LaserScan msg with four points while rotating the turret
*/
#define PI 3.14159265359

float ANGLE_MIN = 0; //PI/2 is straight forward
float ANGLE_MAX = PI;
float ANGLE_INCREMENT; //rad == 4 deg
int TIME_INCREMENT = 30000; // us WHY CAN"T GO OVER 30000? OVERFLOW?
long SCAN_TIME; // us
int N;
#define RANGE_MIN 0.03 //meters
#define RANGE_MAX 2.00 //meters
#define RETURN_TIME 1000000 //us

#define SERVO_PIN 3

//#define SERIALDEBUG;


//instantiate a handle for the node
#define ROSSERIAL // ros connectivity
#ifdef ROSSERIAL
ros::NodeHandle n;
//publisher
sweeper_interface::laser_range scan_msg;
ros::Publisher scan_pub("/sweeper/rawLaser", &scan_msg);
#endif

void setup() {



#ifdef SERIALDEBUG
  Serial.begin(115200);
  Serial.println("Serial Debug Enabled: Running Setup");
#endif

  //initialize Rosserial
#ifdef ROSSERIAL
  initComms();
#endif

  //init laser and I2C
  Wire.begin();
  laser.init();
  laser.setTimeout(500);

  //calculate some paramters
  ANGLE_INCREMENT = (ANGLE_MAX - ANGLE_MIN) / N;
  SCAN_TIME = (long)N * TIME_INCREMENT + RETURN_TIME;

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  laser.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

  // set timing budget to
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


  //get parameters
  if (! n.getParam("~angle_min", &ANGLE_MIN)) {
    ANGLE_MIN = 0;
  }

  if (! n.getParam("~angle_max", &ANGLE_MAX)) {
    ANGLE_MAX = PI;
  }

  if (! n.getParam("~num_pts", &N)) {
    N = 64;
  }

/* NOT WORKING: HANGS. WHY?
  if (! n.getParam("~time_increment", &TIME_INCREMENT)) {
    TIME_INCREMENT = 30000;
  }
*/

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
    scan_msg.range = range;
    scan_msg.seq = N - i - 1;
    //publish the scan data
    scan_pub.publish(&scan_msg); //mousePose_msg is published to topic mouse_odo
    n.spinOnce();
#endif
#ifdef SERIALDEBUG
    //  Serial.println(range);
#endif

    laserServo.write((int)((ANGLE_MIN + i * ANGLE_INCREMENT) * 180 / PI));
    delay(10);

    //delay until TIME_INCREMENT is up
    while (micros() - pingStart < TIME_INCREMENT) {}
  }
  laserServo.write((int)(ANGLE_MIN * 180 / PI));

}

//main loop
void loop() {

  unsigned long scanStart = micros();
#ifdef SERIALDEBUG
  Serial.println("getting new scan");
#endif
  get_scan();

  //spin until the servo returns for another sweep

  while (micros() - scanStart < SCAN_TIME)  {

#ifdef ROSSERIAL
    n.spinOnce();
#endif
  }
#ifdef SERIALDEBUG
  Serial.println("Scan Aquired");
#endif

}
