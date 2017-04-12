#include <Servo.h>
#include <ros.h>
//a header file for each msg type must be included
#include <sweeper_interface/Minimal_PointCloud.h>
#include <std_msgs/Bool.h>

/*
  This program publishes a simplified pointcloud msg with four points without rotating the turret
*/


//scan parameters
#define SCAN_RES 4 //scans per 360 deg
#define RANGE_MIN 0.02//m
#define RANGE_MAX 2//m
#define AVG_SAMPLE_SIZE 8
unsigned long MAX_FLIGHT_TIME; //micros
unsigned long MIN_FLIGHT_TIME; //micros

//sensor parameters
#define TRIGPIN1 8 //forward
#define TRIGPIN2 7 //right
#define TRIGPIN3 10 //rear
#define TRIGPIN4 9 //left
#define ECHOPIN1 4
#define ECHOPIN2 3
#define ECHOPIN3 6
#define ECHOPIN4 5
const byte trigpin[4] = {TRIGPIN1, TRIGPIN4, TRIGPIN3, TRIGPIN2}; //make it easy to loop through the trig pins (CCW)
const byte echopin[4] = {ECHOPIN1, ECHOPIN4, ECHOPIN3, ECHOPIN2};

//instantiate a handle for the node
ros::NodeHandle turretNode;
//publisher
sweeper_interface::Minimal_PointCloud cloud_msg;
ros::Publisher cloud_pub("/sweeper/TurretCloud", &cloud_msg);


void setup() {

  MAX_FLIGHT_TIME = RANGE_MAX*5826.657;// flight time [us] = RANGE_MAX*2*10^6 [us/s] / SoS [m/s]
  MIN_FLIGHT_TIME = RANGE_MIN*5826.657;

  for (byte y = 0; y < 4; y++) {
    pinMode(trigpin[y], OUTPUT);
    pinMode(echopin[y], INPUT);
  }

  //initialize modules
  initComms();
}

//initialize ros nodes and advertise Publishers, subcribe to topics
void initComms() {
  turretNode.getHardware() -> setBaud(57600);
  turretNode.initNode();
  ros::Time begin = turretNode.now();

  //advertise publishers
  turretNode.advertise(cloud_pub);

  while (!turretNode.connected()) {
    turretNode.spinOnce();
  }
}

//get scan from all four sensors in order
void get_scan() {

  static const float m = (RANGE_MIN - RANGE_MAX) / (RANGE_MAX * (RANGE_MAX - RANGE_MIN));//slope of intensity calculation

  cloud_msg.header.frame_id = "/odom";
  cloud_msg.header.stamp = turretNode.now();

  static unsigned long start = 0;
  static long duration = 0;
  static float distance = 0;
  static long temp = 0;

  //get a result from each sensor and put it into the scan array
  for (int i = 0; i < SCAN_RES ; i++) {

    duration = 0;
    distance = 0 ;
    start = micros();

    for (int j = 0; j < AVG_SAMPLE_SIZE; j++) {
      //query the sensor
      digitalWrite(trigpin[i], HIGH);
      delayMicroseconds(20);
      digitalWrite(trigpin[i], LOW);
      delayMicroseconds(10);
      temp =  pulseIn(echopin[i], HIGH, MAX_FLIGHT_TIME);
      if(temp ==0 ){
        temp = MAX_FLIGHT_TIME +1;
      }
      duration += temp;
    }

    duration /= AVG_SAMPLE_SIZE;

    //only process if the time of flight is greater than that required for minimum distance
    if (duration > MIN_FLIGHT_TIME)
    {
      distance = duration * 0.0001716; //duration [us]* 343.2 [m/s] /(2 *1000000 [us/s])  [m]
    }
    else distance = RANGE_MAX+1;

    //calculate the x,y position from angle and distance
    cloud_msg.x[i] = distance * cos(i * 1.57079632679);
    cloud_msg.y[i] = distance * sin(i * 1.57079632679);
    if (distance > 0) cloud_msg.intensity[i] = m * (distance - RANGE_MIN) + 1;
    else cloud_msg.intensity[i] = 0;

    //do nothing until the max ray time is reached
    //while (micros() - start < MAX_FLIGHT_TIME * AVG_SAMPLE_SIZE) {}
  }

}


//main loop
void loop() {

  static unsigned long max_period = SCAN_RES * MAX_FLIGHT_TIME * AVG_SAMPLE_SIZE;
  unsigned long scanStart = micros();

  get_scan();
  cloud_pub.publish(&cloud_msg); //mousePose_msg is published to topic mouse_odo

  /* while (micros() - scanStart < max_period)  {
     turretNode.spinOnce();
    }*/

  turretNode.spinOnce();
}
