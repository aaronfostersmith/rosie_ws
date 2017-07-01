#define ENA 4
#define ENB 5

#define IN1 31
#define IN2 33
#define IN3 35
#define IN4 37

void setup() {
  // put your setup code here, to run once
  //Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  digitalWrite(ENA, HIGH);
  digitalWrite(IN2 , HIGH);
  digitalWrite(IN1, LOW);

  digitalWrite(ENB, HIGH);
  digitalWrite(IN4 , LOW);
  digitalWrite(IN3, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:

}


