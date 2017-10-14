//*************Motor Variables***********************
#include "Motor_Control.h"
int ticks[2] = {0, 0};
//gains for encoder tuning
const int TMR1preload = 53036; //65536-16MHz/64/24Hz
const int encPeriod = 40; //ms
long lastEncUpdate = 0;
Motor_Control motorControl; //motor control object

//**************IMU Variables*****************
#include "Imu.h"
Imu gy_85;
float imuContainer[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0,};
const int imuPeriod = 40; //ms
long lastImuUpdate = 0;

//**************Rosserial Variables and Objects******************
#include <ros.h>
//a header file for each msg type must be included
#include <geometry_msgs/Twist.h> //subscribed velocity  form: vector[3] linear angular
#include <sweeper_interface/Imu_bytes.h>
#include <sweeper_interface/Ticks.h>

const int spinPeriod = 25;
const int paramUpdatePeriod = 5000;

//tracking for timing
long lastParam = 0;
long lastSpin = 0;


//instantiate a handle for the node
ros::NodeHandle arduinoNode;

//instantiate the publishers and subscribers
//publishers
sweeper_interface::Imu_bytes IMU_msg;
ros::Publisher IMU_pub("/sweeper/imu", &IMU_msg);

sweeper_interface::Ticks encTicks_msg; //instantiate a Ticks2D object from geometry_msgs called geometry_msg
ros::Publisher enc_odo("/sweeper/ticks", &encTicks_msg); //instantiate a Publisher with topic enc_odo


//subscribers
void velocityCb(const geometry_msgs::Twist& msg); //declare the callback function
ros::Subscriber<geometry_msgs::Twist> velocity("cmd_vel", &velocityCb); //instantiate a subsciber tagged "velocity" on topic cmd_vel


//**************************encoder ISRs********************************
void ISR_right() {
  motorControl.service_right_enc();

}

void ISR_left() {
  motorControl.service_left_enc();

}

void setup() {

  //***************************************Motor Initialization***********************************************
  //attach interrupts for left and right encoders
  attachInterrupt(digitalPinToInterrupt(motorControl.get_left_enc()), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(motorControl.get_right_enc()), ISR_right, RISING);

  //initialize timer 3 (16bit) : interrupt frequency (Hz) for PID loop
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = TMR1preload;            // preload timer 65536-16MHz/64/24Hz using timer1 means pin 12, 11 do not have pwm
  TCCR1B |= (1 << CS10) | (1 << CS11) |  (0 << CS12);    // 64 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();

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
  arduinoNode.advertise(enc_odo);
  // arduinoNode.advertise(mouse_odo);
  arduinoNode.advertise(IMU_pub);
  //  arduinoNode.advertise(battVoltage_pub);
  //  arduinoNode.advertise(battCurrent_pub);

  //subscribe to incoming topics
  arduinoNode.subscribe(velocity);

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

    //get encoder gains values as a parameter
    float mouseGains[2] = {1, 1}; //gains
    if (!arduinoNode.getParam("~mouse_gains", mouseGains, 2)) {
      mouseGains[0] = 1;
      mouseGains[1] = 1;
      arduinoNode.logwarn("Failed to get Mouse Gains, Using defaults");
    }

    //set gains for mouse and encoders
    motorControl.set_enc_gains(encGains[0], encGains[1]);
    mouseOdo.set_mouse_gains(mouseGains[0], mouseGains[1]);
  */
}

//callback function for velocity subscription
void velocityCb(const geometry_msgs::Twist& msg) {
  //map linear and rotational vectors to wheel velocities [mm/s]
  motorControl.set_tar_spd((msg.linear.x * 1000 - msg.angular.z * 95), (msg.linear.x * 1000 + msg.angular.z * 95)); //left_speed_out = cmd_vel.linear.x - cmd_vel.angular.z*ROBOT_WIDTH/2 ,right_speed_out = cmd_vel.linear.x + cmd_vel.angular.z*ROBOT_WIDTH/2
}

//timer interrupt for motor PID update
ISR(TIMER1_OVF_vect)
{
  TCNT1 = TMR1preload;            // preload timer
  motorControl.update_PID();
}

void loop() {

  long timeNow = millis();
  if (timeNow - lastEncUpdate >= encPeriod) {
    //publish pose data calculated from encoder data
    motorControl.get_EncTicks(ticks);
    encTicks_msg.stamp = arduinoNode.now();
    encTicks_msg.l = ticks[0]; //left
    encTicks_msg.r = ticks[1]; //right
    enc_odo.publish(&encTicks_msg); //encTicks_msg is published to topic enc_odo
    lastEncUpdate = timeNow;
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
    lastImuUpdate = timeNow;
  }


  /*
    timeNow = millis();
    if (timeNow - lastParam >= paramUpdatePeriod) {

      //get encoder gains values as a parameter
      float encGains[2] = {1, 1}; //gaintheta,gainxy
      if (!arduinoNode.getParam("/sweeper/enc_gains", encGains, 2)) {
        encGains[0] = 1;
        encGains[1] = 1;
      }

      //get encoder gains values as a parameter
      float mouseGains[2] = {1, 1}; //gains
      if (!arduinoNode.getParam("/sweeper/mouse_gains", mouseGains, 2)) {
        mouseGains[0] = 1;
        mouseGains[1] = 1;

      }
      lastParam = timeNow;
    }

  */

  timeNow = millis();
  if (timeNow - lastSpin >= spinPeriod) {
    arduinoNode.spinOnce();
    lastSpin = timeNow;
  }
}

