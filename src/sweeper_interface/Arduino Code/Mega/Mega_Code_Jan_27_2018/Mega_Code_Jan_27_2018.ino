//*************Motor Variables***********************

#include <Rotary.h>
#define LENCA 50
#define LENCB 51
#define RENCA A8
#define RENCB A9
int16_t countL = 0;
int16_t countR = 0;

Rotary rL = Rotary(LENCA, LENCB);
Rotary rR = Rotary(RENCA, RENCB);

const unsigned int encPeriod = 25; 
unsigned long lastEncUpdate = 0;

#include "Motor_Control.h"
Motor_Control lMtr; //motor control object
#define ENL 9
#define IN1L 32
#define IN2L 34
Motor_Control rMtr;
#define ENR 8
#define IN1R 28
#define IN2R 30

//**************IMU Variables*****************
#include "Imu.h"
Imu gy_85;
float imuContainer[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0,};
const unsigned int imuPeriod = 100; //us
unsigned long lastImuUpdate = 0;

//**************Rosserial Variables and Objects******************
#include <ros.h>
//a header file for each msg type must be included
#include <std_msgs/Float32.h> //subscribed velocity  form: vector[3] linear angular
#include <sweeper_interface/Imu_bytes.h>
#include <std_msgs/Int16.h>

const unsigned int spinPeriod = 100; //ms
const unsigned int paramUpdatePeriod = 1000;

//tracking for timing
unsigned long lastParam = 0;
unsigned long lastSpin = 0;


//instantiate a handle for the node
ros::NodeHandle arduinoNode;

//instantiate the publishers and subscribers
//publishers
sweeper_interface::Imu_bytes IMU_msg;
ros::Publisher IMU_pub("/sweeper/imu", &IMU_msg);

std_msgs::Int16 lwheel_msg; //instantiate an int16 msg called lwheel_msg
ros::Publisher lwheel_pub("/sweeper/lwheel", &lwheel_msg); //instantiate a Publisher with topic ../lwheel to publish ticks

std_msgs::Int16 rwheel_msg; //instantiate an int16 msg called rwheel_msg
ros::Publisher rwheel_pub("/sweeper/rwheel", &rwheel_msg); //instantiate a Publisher with topic ../rwheel to publish ticks

//subscribers
void lmtr_pwr_CB(const std_msgs::Float32& msg); //declare the callback function
ros::Subscriber<std_msgs::Float32> lmtr_pwr_sub("/sweeper/l_motor_cmd", &lmtr_pwr_CB);
void rmtr_pwr_CB(const std_msgs::Float32& msg); //declare the callback function
ros::Subscriber<std_msgs::Float32> rmtr_pwr_sub("/sweeper/r_motor_cmd", &rmtr_pwr_CB);

//**************************encoder ISRs********************************


void setup() {

  //***************************************Motor Initialization***********************************************
  lMtr.init(ENL, IN1L, IN2L);
  rMtr.init(ENR, IN1R, IN2R);

  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3);
  sei();

  //****************************************IMU Initialization*************************************
  gy_85 = Imu(5.0);
  gy_85.isLpfOn = false;
  gy_85.Calibrate();

  //****************************************Rosserial Initialization*******************************
  initComms();




}

//initialize ros nodes and advertise Publishers, subcribe to topics
void initComms() {
  arduinoNode.getHardware() -> setBaud(115200);
  arduinoNode.initNode();

  //advertise publishers
  arduinoNode.advertise(lwheel_pub);
  arduinoNode.advertise(rwheel_pub);
  arduinoNode.advertise(IMU_pub);

  //subscribe to incoming topics
  arduinoNode.subscribe(lmtr_pwr_sub);
  arduinoNode.subscribe(rmtr_pwr_sub);

  //process callbacks
  while (!arduinoNode.connected()) arduinoNode.spinOnce();

  /*
    arduinoNode.loginfo("MEGA: attempting to get parameters");

    //get encoder gains values as a parameter
    float encGains[2] = {1, 1}; //gaintheta,gainxy
    if (!arduinoNode.getParam("~enc_gains", encGains, 2)) {
      encGains[0] = 1;
      encGains[1] = 1;
      arduinoNode.logwarn("Failed to get Encoder Gains, Using defaults");
    }
  */
}

//callbacks for motor power subscription
void lmtr_pwr_CB(const std_msgs::Float32& msg) {
  lMtr.set_pwr(msg.data);
}
void rmtr_pwr_CB(const std_msgs::Float32& msg) {
  rMtr.set_pwr(msg.data);
}

void loop() {

  long timeNow = millis();
  if (timeNow - lastEncUpdate >= encPeriod) {

    //publish pose data calculated from encoder data
    lwheel_msg.data = countL;
    rwheel_msg.data = countR;
    lwheel_pub.publish(&lwheel_msg);
    rwheel_pub.publish(&rwheel_msg);

    arduinoNode.spinOnce();
    lastSpin = millis();
    lastEncUpdate = lastSpin;
  }

  timeNow = millis();
  if (timeNow - lastImuUpdate >= imuPeriod) {

    gy_85.update_imu(imuContainer);
    IMU_msg.header.stamp = arduinoNode.now();
    IMU_msg.magnetic_field[0] = imuContainer[0];
    IMU_msg.magnetic_field[1] = imuContainer[1];
    IMU_msg.magnetic_field[2] = imuContainer[2];
    IMU_msg.angular_velocity[0] = imuContainer[3];
    IMU_msg.angular_velocity[1] = imuContainer[4];
    IMU_msg.angular_velocity[2] = imuContainer[5];
    IMU_msg.linear_acceleration[0] = imuContainer[6];
    IMU_msg.linear_acceleration[1] = imuContainer[7];
    IMU_msg.linear_acceleration[2] = imuContainer[8];
    IMU_pub.publish(&IMU_msg);

    arduinoNode.spinOnce();
    lastSpin = millis();
    lastImuUpdate = lastSpin;
  }


  timeNow = millis();
  if (timeNow - lastSpin >= spinPeriod) {
    arduinoNode.spinOnce();
    lastSpin = millis();
  }
}


ISR(PCINT0_vect) {
  unsigned char result = rL.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) {
    countL ++;
  }
  else if (result == DIR_CCW) {
    countL --;
  }
}
ISR(PCINT2_vect) {
  unsigned char result = rR.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) {
    countR ++;
  }
  else if (result == DIR_CCW) {
    countR --;
  }
}

