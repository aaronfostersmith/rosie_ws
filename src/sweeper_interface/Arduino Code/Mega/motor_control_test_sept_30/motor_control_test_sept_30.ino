//*************Motor Variables***********************
#include "Motor_Control.h"
float encPose[3] = {0, 0, 0};

const int TMR1preload = 53036; //65536-16MHz/64/24Hz
const int encPeriod = 50; //ms
long lastEncUpdate = 0;
Motor_Control motorControl; //motor control object

//**************************encoder ISRs********************************
void ISR_right() {
  motorControl.service_right_enc();

}

void ISR_left() {
  motorControl.service_left_enc();

}

void setup() {
  Serial.begin(9600);

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

  motorControl.set_motor_spd(LEFTMTR, 31);

}

//timer interrupt for motor PID update
ISR(TIMER1_OVF_vect)
{
  TCNT1 = TMR1preload;            // preload timer
  motorControl.update_PID();
}

void loop() {
  // put your main code here, to run repeatedly:

}
