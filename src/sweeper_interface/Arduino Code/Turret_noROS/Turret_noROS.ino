#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define MOTOR_PIN 5
#define ENC_PIN 2

double Setpoint, Input, Output;
double Kp, Ki, Kd;
PID drivePID(&Input, &Output, &Setpoint, 20, 8, 3, DIRECT);
PID_ATune driveATune(&Input, &Output);

void setup() {
  Serial.begin(9600);
  pinMode(ENC_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), ENC_ISR, CHANGE);

  Setpoint = 1.0; //Hz
  Input = 0;

  drivePID.SetOutputLimits(84, 255);
  drivePID.SetSampleTime(1000 / (Setpoint * 8));
  drivePID.SetMode(AUTOMATIC);
  //tune();
}

void tune() {
  driveATune.SetControlType(1); //PID
  driveATune.SetOutputStep(64);
  driveATune.SetNoiseBand(0.2);
  //  driveATune.SetLookbackSec(2);
  Output = 175;
  while (!driveATune.Runtime()) {

    Serial.println(Input);
    analogWrite(MOTOR_PIN, Output);
    delay(30);
  }
  Kp = driveATune.GetKp();
  Ki =  driveATune.GetKi();
  Kd =  driveATune.GetKd();
  drivePID.SetTunings(Kp, Ki, Kd);
  Serial.println("Tuning Completed: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.println(Kd);

}

long lastime = 0;

void ENC_ISR() {
  long thistime = micros();
  Input = 1000000.0 / (8 * (thistime - lastime));
  lastime = thistime;
}

// the loop routine runs over and over again forever:
long lastcalc = 0;
void loop() {
  drivePID.Compute();
  analogWrite(MOTOR_PIN, Output);
    Serial.print(Input);
    Serial.print(", ");
    Serial.print(Output/128);
    Serial.print(", ");
    Serial.println(Setpoint);

  delay(15);
}
