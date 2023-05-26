#include "mpu_ahrs_ekf.h"

unsigned long Time;

void setup() {
  Serial.begin(2000000);
  imu_setup();
  ahrs_setup();
}

void loop() {
  Time = millis();
  
  ahrs_update();
  Serial.print(q[0]);
  Serial.print(",");
  Serial.print(q[1]);
  Serial.print(",");
  Serial.print(q[2]);
  Serial.print(",");
  Serial.println(q[3]);

  while(millis()-Time<4);
}
