#include "Arduino.h"
#include "MAX471.h"

MAX471::MAX471(int currentPin, int voltagePin)
: mCurPin(currentPin)
, mVolPin(voltagePin)
{
}

void MAX471::start()
{
    pinMode(mCurPin, INPUT);
  pinMode(mVolPin, INPUT);
}

float MAX471::current()
{
  return analogRead(mCurPin);
}

float MAX471::voltage()
{
  return analogRead(mVolPin);
}
