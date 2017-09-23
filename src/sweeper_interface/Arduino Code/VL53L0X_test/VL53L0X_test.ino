#include <VL53L0X.h>
#include <Wire.h>

#define TIMING_BUDGET 20000
VL53L0X laser;
//#define LONG_RANGE


void setup() {

  Serial.begin(115200);

//init laser and I2C
  Wire.begin();
  laser.init();
  laser.setTimeout(TIMING_BUDGET);

  // set timing budget to
  laser.setMeasurementTimingBudget(TIMING_BUDGET);


}

void loop() {
  long len = micros();
  int range = laser.readRangeSingleMillimeters();
  if (laser.timeoutOccurred()|| range> 2000) {
    range = 0;
  } 
  
  while(micros() - len < TIMING_BUDGET+5000){}

  len= micros()-len;
  
    Serial.print(range);
    Serial.print(",");
    Serial.println(len);
    
}
