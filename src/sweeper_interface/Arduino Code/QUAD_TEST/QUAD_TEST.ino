#define ENA 12
#define ENB 13

#define IN1 28
#define IN2 30
#define IN3 32
#define IN4 34

#define LENCA 50
#define LENCB 51
#define RENCA A8
#define RENCB A9


#include <Rotary.h>

Rotary rL = Rotary(LENCA, LENCB);
Rotary rR = Rotary(RENCA, RENCB);

long countL=0;
long countR=0;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3);
  sei();

}


void loop() {

Serial.print(countL);
Serial.print(", ");
Serial.println(countR);
delay(20);
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
