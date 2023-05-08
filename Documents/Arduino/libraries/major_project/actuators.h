#include <Arduino.h>
#include <Servo.h>


#define vane1_pin 2
#define vane2_pin 3
#define vane3_pin 4
#define vane4_pin 5
#define ESC_pin 6

#define vane1_bias 89
#define vane2_bias 88
#define vane3_bias 98
#define vane4_bias 65
#define ESC_bias 1000

#define vane_init_delay_time 1000  //in ms 
#define ESC_init_delay_time 1000  //in ms


extern int vane1_angle;
extern int vane2_angle;
extern int vane3_angle;
extern int vane4_angle;

extern int ESC_value;

void actuators_init();
void actuators_write();
int vane_angle_saturator(int vane_angle, int yaw_controlInput_bias);
int ESC_value_saturator(int ESC_value);