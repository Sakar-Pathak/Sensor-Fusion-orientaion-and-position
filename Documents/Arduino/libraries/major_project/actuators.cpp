#include "actuators.h"

Servo vane1;
Servo vane2;
Servo vane3;
Servo vane4;
Servo ESC;


int vane1_angle = 0;
int vane2_angle = 0;
int vane3_angle = 0;
int vane4_angle = 0;

int ESC_value = 0;

void actuators_init()
{
  vane1.attach(vane1_pin);
    vane1.write(vane1_bias);
        delay(vane_init_delay_time);

  vane2.attach(vane2_pin);
    vane2.write(vane2_bias);
        delay(vane_init_delay_time);
  
  vane3.attach(vane3_pin);
    vane3.write(vane3_bias);
        delay(vane_init_delay_time);

  vane4.attach(vane4_pin);
    vane4.write(vane4_bias);
        delay(vane_init_delay_time);

  ESC.attach(ESC_pin,1000,2000);
    ESC.write(ESC_bias);
        delay(ESC_init_delay_time);
}

void actuators_write()
{
  vane1.write(vane1_angle + vane1_bias);
  vane2.write(vane2_angle + vane2_bias);
  vane3.write(vane3_angle + vane3_bias);
  vane4.write(vane4_angle + vane4_bias);
  ESC.write(ESC_value );
}

int vane_angle_saturator(int vane_angle, int yaw_controlInput_bias)
{
  if (vane_angle > 38 + yaw_controlInput_bias)
    vane_angle = 38 + yaw_controlInput_bias;
  else if (vane_angle < -38 + yaw_controlInput_bias)
    vane_angle = -38 + yaw_controlInput_bias;
  
  return vane_angle;
}

int ESC_value_saturator(int ESC_value)
{
  if(ESC_value > 1660)  //1660
    ESC_value = 1660;
  else if(ESC_value < 1000)   //1095
    ESC_value = 1000;

  return ESC_value;
}